import torch
from quality_environment import QualityEnv, QualityCfg
from environment import AS2WEnv, tensor, RADIUS

class ArcEnv(QualityEnv):
    def __init__(self,cfg,**kwargs):
        super().__init__(cfg,**kwargs)
        self.latent=torch.zeros(self.num_envs,16,device=self.device)
        self.physical=torch.zeros_like(self.latent)
        self.progress=0.
        self.reference_q=None
        self.reference_height=None

    def _step_strength(self):
        return torch.zeros(self.num_envs,device=self.device)

    def _sample_commands(self,ids):
        n=len(ids)
        mode=torch.randint(0,5,(n,),device=self.device)
        speed=.15+torch.rand(n,device=self.device)*(.20+.35*self.progress)
        speed=torch.where(mode==2,-speed,speed)
        curvature=.3+torch.rand(n,device=self.device)*(.5+.7*self.progress)
        yaw=torch.where(mode==3,speed*curvature,torch.where(mode==4,-speed*curvature,0.))
        command=torch.stack((speed,torch.zeros_like(speed),yaw),1)
        command[mode==0]=0.
        self.command[ids]=command

    def _pre_physics_step(self,latent):
        self.latent.copy_(latent)
        # Resample once before calculating feedforward. Parent resampling uses this guard.
        interval=round(2.5/self.step_dt)
        ids=(self.episode_length_buf.remainder(interval)==0).nonzero(as_tuple=True)[0]
        ArcEnv._sample_commands(self,ids)
        if self.exercise is not None:
            self.command[:]=self.exercise
        bounded=latent.tanh()
        side=self.command.new_tensor([.1654,-.1654,.1654,-.1654])
        wheel=(self.command[:,:1]-self.command[:,2:3]*side)/RADIUS
        wheel=(wheel+2.*bounded[:,12:]).clamp(-12.,12.)
        self.physical.copy_(torch.cat((.6*bounded[:,:12],wheel/12.),1))
        sample=self._sample_commands
        self._sample_commands=lambda ids: None
        try:
            super()._pre_physics_step(self.physical)
        finally:
            self._sample_commands=sample

    def _get_rewards(self):
        reward=AS2WEnv._get_rewards(self)
        d=self.robot.data
        q=tensor(d.joint_pos)[:,self.leg_ids]
        dq=tensor(d.joint_vel)[:,self.leg_ids]
        loads=tensor(self.sensor.data.net_normal_forces_w)[:,self.foot_sensors,2].clamp_min(0.)
        still=torch.exp(-self.command[:,0].square()/.01-self.command[:,2].square()/.01)
        ref=self.leg_action_center if self.reference_q is None else self.reference_q
        pose=((q-ref).abs()-.08).clamp_min(0.).square().mean(1)/.04
        gravity=tensor(d.projected_gravity_b)
        tilt=(gravity[:,:2].norm(dim=1)-.07).clamp_min(0.).square()/.04
        deficit=((8.-loads)/8.).clamp_min(0.).square().mean(1)
        height=tensor(d.root_pos_w)[:,2]-self.terrain.env_origins[:,2]
        target=.37 if self.reference_height is None else self.reference_height
        hcost=((height-target)/.03).square().clamp_max(4.)
        # Contact-point slip includes wheel angular velocity.
        vel=tensor(d.body_link_lin_vel_w)[:,self.wheel_bodies]
        omega=tensor(d.body_link_ang_vel_w)[:,self.wheel_bodies]
        quat=tensor(d.body_link_quat_w)[:,self.wheel_bodies]
        x,y,z,w=quat.unbind(-1)
        axis=torch.stack((2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w)),-1)
        down=torch.zeros_like(axis);down[...,2]=-1.
        radial=down-(down*axis).sum(-1,keepdim=True)*axis
        radial=radial/radial.norm(dim=-1,keepdim=True).clamp_min(.05)
        cp=vel+torch.cross(omega,RADIUS*radial,dim=-1)
        slip=((cp*axis).sum(-1).square()*(loads>=3)).mean(1)
        latent_cost=(self.latent.abs()-1.5).clamp_min(0.).square().mean(1)
        cost=(.5+still)*pose+1.5*tilt+1.0*deficit+.15*hcost+.15*slip+.01*(1+4*still)*dq.square().mean(1)+.03*latent_cost
        self.metrics={"pose":pose.mean(),"tilt":tilt.mean(),"support_deficit":deficit.mean(),"slip":slip.mean()}
        return reward-self.step_dt*cost
