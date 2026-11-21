import torch
from isaaclab import sim
from arc_environment import ArcEnv, QualityCfg
from environment import AS2WEnv, tensor, RADIUS

HEIGHT=.05
EDGE=.7
END=2.7

class StairEnv(ArcEnv):
    def _setup_scene(self):
        super()._setup_scene()
        mat=sim.RigidBodyMaterialCfg(static_friction=.8,dynamic_friction=.65)
        for i,(x,y,z) in enumerate(self.scene.env_origins.cpu().tolist()):
            block=sim.CuboidCfg(size=(END-EDGE,2.,HEIGHT),
                collision_props=sim.CollisionPropertiesCfg(),physics_material=mat)
            block.func(f"/World/StairCourse/lane_{i}/step",block,
                       translation=(x+(EDGE+END)/2,y,z+HEIGHT/2))

    def __init__(self,cfg,**kwargs):
        super().__init__(cfg,**kwargs)
        self.landed=torch.zeros(self.num_envs,4,device=self.device,dtype=torch.bool)
        self.success=torch.zeros(self.num_envs,device=self.device,dtype=torch.bool)
        self.dwell=torch.zeros(self.num_envs,device=self.device)
        self.completed_count=0
        self.episode_count=0

    def _sample_commands(self,ids):
        self.command[ids]=self.command.new_tensor([.25,0.,0.])

    def leg_gain(self):
        # Preserve the original action mapping initially, expand thigh/calf over 1000 updates.
        gain=self.command.new_tensor([.6,1.,1.]*4)
        return .6+(gain-.6)*min(1.,self.progress*2.)

    def ground(self,position):
        local=position-self.scene.env_origins[:,None,:]
        return ((local[...,0]>=EDGE)&(local[...,0]<=END)&(local[...,1].abs()<1.)).float()*HEIGHT

    def _pre_physics_step(self,action):
        super()._pre_physics_step(action)
        # Stop after all wheel centers have cleared the edge by a wheel radius.
        wheels=tensor(self.robot.data.body_link_pos_w)[:,self.wheel_bodies]
        finish=((wheels[...,0]-self.scene.env_origins[:,None,0]).amin(1)>EDGE+.12)
        self.command[finish]=0.
        self.target[finish,12:]*=(1.-self.alpha_wheel)
        self.applied[finish,12:]=self.target[finish,12:]

    def _get_rewards(self):
        base=AS2WEnv._get_rewards(self)
        d=self.robot.data
        wheels=tensor(d.body_link_pos_w)[:,self.wheel_bodies]
        ground=self.ground(wheels)
        force=tensor(self.sensor.data.net_normal_forces_w)
        load=force[:,self.foot_sensors,2].clamp_min(0.)
        grounded=(load>3.)&((wheels[...,2]-self.scene.env_origins[:,None,2]-ground-RADIUS).abs()<.025)
        upright=-tensor(d.projected_gravity_b)[:,2]>.94
        stable=upright&(tensor(d.root_ang_vel_b).norm(dim=1)<1.5)
        landing=grounded&(ground>0)&stable[:,None]
        fresh=landing&~self.landed
        self.landed |= landing
        x=wheels[...,0]-self.scene.env_origins[:,None,0]
        complete=self.landed.all(1)&(ground>0).all(1)&(x.amin(1)>EDGE+.12)&(grounded.sum(1)>=3)&stable
        self.dwell=torch.where(complete,self.dwell+self.step_dt,0.)
        self.success |= self.dwell>=.5
        local_height=tensor(d.root_pos_w)[:,2]-self.scene.env_origins[:,2]-ground.mean(1)
        q=tensor(d.joint_pos)[:,self.leg_ids]
        ref=self.leg_action_center if self.reference_q is None else self.reference_q
        near=((x-EDGE).abs().amin(1)<.25)|(ground.amax(1)>ground.amin(1))
        relief=torch.where(near,.4,1.)
        pose=((q-ref).abs()-.10).clamp_min(0.).square().mean(1)/.04
        tilt=tensor(d.projected_gravity_b)[:,:2].square().sum(1)
        # Crossing permits one unloaded wheel, while landing still requires geometric support.
        support=(3-grounded.sum(1)).clamp_min(0.).float().square()
        collision=(force[:,self.nonwheel_sensors].norm(dim=-1)>10.).any(1).float()
        moving=self.command[:,0]>.1
        stall=(.5*self.command[:,0]-tensor(d.root_lin_vel_b)[:,0]).clamp_min(0.)/.25
        height_target=.3415 if self.reference_height is None else self.reference_height
        hcost=((local_height-height_target)/.04).square().clamp_max(4.)
        latent=(self.latent.abs()-1.5).clamp_min(0.).square().mean(1)
        cost=.5*relief*pose+1.5*tilt+support+collision+.15*hcost+.5*stall*moving+.03*latent
        self.metrics={"stair_success":self.success.float().mean(),"landed_wheels":self.landed.float().sum(1).mean(),
                      "pose":pose.mean(),"nonwheel_collision":collision.mean()}
        return base-self.step_dt*cost+.5*fresh.float().mean(1)

    def _reset_idx(self,ids):
        if ids is None: ids=self.rows
        if hasattr(self,'success'):
            self.completed_count+=int(self.success[ids].sum())
            self.episode_count+=len(ids)
            self.landed[ids]=False;self.success[ids]=False;self.dwell[ids]=0.
        super()._reset_idx(ids)
