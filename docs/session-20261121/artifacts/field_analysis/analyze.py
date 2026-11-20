from pathlib import Path
import csv,json,hashlib
import numpy as np
B=Path(r'X:\Items\as2w\AS2-W_CPRO'); O=Path('field_analysis')
S=B/'部署测试/04_调试记录/logs/field_sessions'
slots=[3,0,9,6,4,1,10,7,5,2,11,8,13,12,15,14]; signs=np.array([-1]*4+[1]*12)
names=[f'{leg}_{part}' for part in ['hip','thigh','calf','wheel'] for leg in ['FL','FR','RL','RR']]
def pitch(q):
 q=q/np.linalg.norm(q,axis=1)[:,None];w,x,y,z=q.T
 return np.degrees(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
def stats(x):return dict(mean=float(np.mean(x)),p05=float(np.quantile(x,.05)),p95=float(np.quantile(x,.95)),min=float(np.min(x)),max=float(np.max(x)))
result={}
for sess in ['20261117-184650_lp5w-joystick','20261117-191631_lp5w-joystick','20261117-191852_lp5w-joystick']:
 p=S/sess
 lows=[]
 for r in csv.reader((p/'low_capture_fast.log').open()):
  if r[0]=='LOW_SAMPLE' and len(r)>100:lows.append(list(map(float,r[1:])))
 # inspect schema and map timestamp/tick reliably
 a=np.array(lows);print(sess,'lowshape',a.shape,'first',a[0,:16].tolist())
 cmd=[]
 for r in csv.reader((p/'debug_lowcmd.csv').open()):
  if r[0]=='DEBUG_LOWCMD':cmd.append(list(map(float,r[1:])))
 c=np.array(cmd); motors=c[:,4:].reshape(-1,16,7)
 lowtick=a[:,3]; ctick=c[:,1]
 ix=np.searchsorted(ctick,lowtick,side='right')-1; valid=ix>=0;ix=np.maximum(ix,0)
 m=motors[ix]; state=c[ix,2];q=a[:,13:].reshape(len(a),-1,6)[:,slots,0]*signs;dq=a[:,13:].reshape(len(a),-1,6)[:,slots,1]*signs; tau=a[:,13:].reshape(len(a),-1,6)[:,slots,3]*signs
 target=m[:,:,2]*signs;targetdq=m[:,:,3]*signs
 print('states',[(v,int((state==v).sum())) for v in np.unique(state)])
 report={'low_frames':len(a),'states':{}}
 for st in np.unique(state):
  mask=valid&(state==st)
  if mask.sum()<200:continue
  # remove transitions by trimming 1.5 s of each contiguous state window
  runs=np.flatnonzero(np.r_[True,np.diff(state)!=0,True]);mask[:]=False
  for l,u in zip(runs[:-1],runs[1:]):
   if state[l]==st:
    mask[l:u]=(a[l:u,0]>a[l,0]+1500)&(a[l:u,0]<a[u-1,0]-1500)&valid[l:u]
  if mask.sum()<20:continue
  report['states'][str(int(st))]={'frames':int(mask.sum()),'pitch':stats(pitch(a[mask,6:10])), 'leg_kp':np.unique(m[mask,:12,4]).tolist(),'leg_kd':np.unique(m[mask,:12,5]).tolist(),'wheel_kd':np.unique(m[mask,12:,5]).tolist(),'wheel_tau_ff_max':float(abs(m[mask,12:,6]).max()), 'joints':{n:{'q':float(q[mask,j].mean()),'target':float(target[mask,j].mean()),'error':float((q-target)[mask,j].mean()),'dq_rms':float(np.sqrt(np.mean(dq[mask,j]**2))),'dq_target_rms':float(np.sqrt(np.mean(targetdq[mask,j]**2))),'tau_rms':float(np.sqrt(np.mean(tau[mask,j]**2)))}for j,n in enumerate(names)}}
 policy=[]
 for r in csv.reader((p/'debug_policy.csv').open()):
  if r[0]!='DEBUG_POLICY' or r[2]!='WALK':continue
  n=int(r[57]); k=58+n;ns=int(r[k]);k+=1+ns
  policy.append((float(r[1]),np.array(r[5:8],float),np.array(r[18:22],float),np.array(r[25:41],float),np.array(r[41:57],float),np.array(r[k+32+64:k+32+128],float).reshape(16,4)))
 if policy:
  t=np.array([r[0] for r in policy])*1e-9; command=np.stack([r[1] for r in policy]);pdq=np.stack([r[4] for r in policy]);pp=pitch(np.stack([r[2] for r in policy]));pq=np.stack([r[3] for r in policy]);ap=np.stack([r[5] for r in policy]);dt=np.r_[np.diff(t),0]
  report['walk']={'frames':len(t),'duration':float(t[-1]-t[0]),'command_abs_distance':float(sum(abs(command[:,0])*dt)),'wheel_encoder_proxy_distance_r086':float(sum(abs(pdq[:,12:].mean(1))*.086*dt)),'wz_max':float(abs(command[:,2]).max()),'all_pitch':stats(pp),'moving_pitch':stats(pp[abs(command[:,0])>.02]) if np.any(abs(command[:,0])>.02) else None,'moving_frames':int((abs(command[:,0])>.02).sum()),'joint_mean_errors':dict(zip(names,(pq-ap[:,:,0]).mean(0).tolist()))}
  np.savez_compressed(O/(sess+'.npz'),t=t,command=command,pitch=pp,q=pq,dq=pdq,applied=ap)
 result[sess]=report
(O/'raw_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({k:{a:b for a,b in v.items() if a!='states'} for k,v in result.items()},ensure_ascii=False,indent=2))
