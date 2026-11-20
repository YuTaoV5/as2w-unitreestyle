import csv,json
from pathlib import Path
import numpy as np
p=Path(r'X:/Items/as2w/AS2-W_CPRO/部署测试/04_调试记录/logs/AS2W官方运控基线数据包_20261119')
out={}
for label,pattern,lo,hi in [('ground','03_*',2,15),('suspended','04_*',2,55)]:
 f=next(p.glob(pattern))/'low_capture_fast.log'
 with f.open() as stream:
  rows=np.array([r[1:110] for r in csv.reader(stream) if len(r)>=110 and r[0]=='LOW_SAMPLE' and lo*1000<=int(r[1])<hi*1000],float)
 slots=[3,4,5,0,1,2,9,10,11,6,7,8]
 q,dq,_,tau,*_=rows[:,13:].reshape(-1,16,6)[:,slots].transpose(2,0,1)
 target=np.tile([-.15,.6,-1.2,.15,.6,-1.2],2)
 pd=200*(target-q)-4*dq
 out[label]=dict(frames=len(rows),q_mean=q.mean(0).reshape(4,3).tolist(),tau_mean=tau.mean(0).reshape(4,3).tolist(),pd_mean=pd.mean(0).reshape(4,3).tolist(),pd_tau_mae=float(abs(pd-tau).mean()))
delta=np.array(out['ground']['tau_mean'])-np.array(out['suspended']['tau_mean'])
out['ground_minus_suspended_tau_nm']=delta.tolist()
out['rear_calf_torque_share_proxy']=float(delta[2:,2].sum()/delta[:,2].sum())
Path(__file__).with_name('official-static-torque.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
