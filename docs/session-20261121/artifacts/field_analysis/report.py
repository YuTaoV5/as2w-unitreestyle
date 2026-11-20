from pathlib import Path
import csv, json, hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

B=Path(r'X:\Items\as2w\AS2-W_CPRO')
O=Path(__file__).resolve().parent
E=B/'部署参考/Rob2Nav-deploy-as2w-lp5-1k-20261117/evidence'
names=[f'{leg}_{part}' for part in ['hip','thigh','calf','wheel'] for leg in ['FL','FR','RL','RR']]
rows=[]; summary={}
fig,ax=plt.subplots(4,2,figsize=(15,12),sharex='col',constrained_layout=True)
for col,sess in enumerate(['20261117-184650_lp5w-joystick','20261117-191852_lp5w-joystick']):
 d=np.load(O/(sess+'.npz'));t=d['t']-d['t'][0];q=d['q'];target=d['applied'][:,:,0];p=d['pitch'];vx=d['command'][:,0]
 active=abs(vx)>.02; end=t>t[-1]-5
 for phase,mask in [('all_walk',np.ones(len(t),bool)),('command_abs_vx_gt_0.02',active),('last_5s',end)]:
  for j in range(12):
   err=q[mask,j]-target[mask,j]
   rows.append([sess,phase,names[j],q[mask,j].mean(),target[mask,j].mean(),err.mean(),np.sqrt(np.mean(err**2))])
 i=p.argmin()
 summary[sess]={'worst_pitch_deg':float(p[i]),'worst_time_from_walk_s':float(t[i]),'vx_at_worst':float(vx[i]),'active_pitch_mean_deg':float(p[active].mean()),'active_pitch_min_deg':float(p[active].min()),'last5_pitch_mean_deg':float(p[end].mean()),'encoder_mean_net_angle_rad':float(np.sum(d['dq'][:,12:].mean(1)*np.r_[np.diff(t),0]))}
 ax[0,col].plot(t,p,label='IMU pitch');ax[0,col].set_title(sess[:15]);ax[0,col].set_ylabel('Pitch (deg)')
 a=ax[0,col].twinx();a.plot(t,vx,color='orange',alpha=.6,label='vx command');a.set_ylabel('vx command (m/s)');a.set_ylim(-.3,.3)
 for j,leg in enumerate(['FL','FR','RL','RR']):
  ax[1,col].plot(t,q[:,8+j]-target[:,8+j],label=leg)
  ax[2,col].plot(t,q[:,j]-target[:,j],label=leg)
  ax[3,col].plot(t,d['dq'][:,12+j],label=leg,alpha=.65,lw=.6)
 ax[1,col].set_ylabel('Calf measured - applied (rad)');ax[2,col].set_ylabel('Hip measured - applied (rad)');ax[3,col].set_ylabel('Wheel measured dq (rad/s)');ax[3,col].set_xlabel('Time from WALK start (s)')
 for a in ax[:,col]:a.grid(alpha=.2);a.legend(loc='upper left',ncol=4,fontsize=8)
fig.savefig(O/'real_joint_comparison.png',dpi=150);plt.close(fig)
with (O/'joint_errors.csv').open('w',newline='',encoding='utf-8-sig') as f:
 w=csv.writer(f);w.writerow(['session','phase','joint','measured_mean_rad','applied_mean_rad','bias_rad','rmse_rad']);w.writerows(rows)

# Check each contiguous FixedStand interval separately; never merge pre/post-walk stand.
segments=[]
for sess in ['20261117-184650_lp5w-joystick','20261117-191631_lp5w-joystick','20261117-191852_lp5w-joystick']:
 root=B/'部署测试/04_调试记录/logs/field_sessions'/sess
 c=np.array([list(map(float,r[1:])) for r in csv.reader((root/'debug_lowcmd.csv').open()) if r[0]=='DEBUG_LOWCMD'])
 a=np.array([list(map(float,r[1:])) for r in csv.reader((root/'low_capture_fast.log').open()) if r[0]=='LOW_SAMPLE' and len(r)>100])
 ix=np.searchsorted(c[:,1],a[:,3],side='right')-1;valid=ix>=0;ix=np.maximum(ix,0);state=c[ix,2]
 qq=a[:,6:10];qq=qq/np.linalg.norm(qq,axis=1)[:,None];w,x,y,z=qq.T;pitch=np.degrees(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
 cuts=np.r_[0,np.flatnonzero(np.diff(state)!=0)+1,len(state)]
 for l,u in zip(cuts[:-1],cuts[1:]):
  if state[l]!=2 or u-l<20:continue
  mask=np.arange(l,u);mask=mask[(a[mask,0]>=a[u-1,0]-1000)&valid[mask]]
  segments.append({'session':sess,'state':'FixedStand','start_elapsed_s':a[l,0]/1000,'end_elapsed_s':a[u-1,0]/1000,'last_1s_pitch_mean':pitch[mask].mean(),'last_1s_leg_kp':np.unique(c[ix[mask],8]).tolist()})
 within=valid&(a[:,3]<=c[-1,1])
 if sess in summary:summary[sess]['lowcmd_tick_age_p99_within_capture_overlap']=float(np.quantile(a[within,3]-c[ix[within],1],.99))

sim=json.loads((E/'mujoco_matrix_lp5_lp31_3states.json').read_text())
sm={}
for k,vs in sim['cases'].items():
 if k=='hold' or 'straight' in k or 'brake' in k:
  sm[k]={'vx_tail_range':[min(v['vx_tail'] for v in vs),max(v['vx_tail'] for v in vs)],'max_total_tilt_deg':float(max(np.degrees(np.arccos(v['upright_min'])) for v in vs))}
hashes={}
for rel in ['部署参考/Rob2Nav-deploy-as2w-lp5-1k-20261117/weights/policy.onnx','部署参考/cpp_deploy-as2w/models/armd-lp5/policy.onnx']:
 hashes[rel]=hashlib.sha256((B/rel).read_bytes()).hexdigest()
output={'real_walk':summary,'fixedstand_intervals':segments,'mujoco_legs_only_lp5':sm,'onnx_sha256':hashes,'notes':['No body odometry: wheel encoder distance is not measured body travel.','No per-joint simulation traces in the cited matrix JSON; no paired joint RMSE asserted.','Simulator tilt derived from upright is total tilt, not signed pitch.','Real sessions filter legs and wheels; Isaac archived run filters legs only.']}
(O/'comparison.json').write_text(json.dumps(output,indent=2),encoding='utf-8')
print(json.dumps(output,indent=2))
