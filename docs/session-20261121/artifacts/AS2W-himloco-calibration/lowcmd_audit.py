import numpy as np,csv,json
from pathlib import Path
p=Path(__file__).parent
result={}
for s in ('20261119-114540','20261119-190857'):
 def read(f,tag,n):
  with f.open() as stream:
   return np.array([r[1:n] for r in csv.reader(stream) if len(r)>=n and r[0]==tag],float)
 c=read(p/f'{s}-debug_lowcmd.csv','DEBUG_LOWCMD',117)
 l=read(p/f'{s}-low_capture_fast.log','LOW_SAMPLE',110)
 b=Path(r'X:/Items/as2w/himloco_field_20261119')/('vx015_walked' if '114540' in s else 'vx010_X_abort_handoff')
 with (b/'debug_policy.csv').open() as f:
  ticks=[int(r[3]) for r in csv.reader(f) if r and r[0]=='DEBUG_POLICY' and r[2]=='HANDOFF']
 c=c[(c[:,1]>=ticks[0])&(c[:,1]<=ticks[-1])]
 lt,ix=np.unique(l[:,3],return_index=True)
 j=np.searchsorted(lt,c[:,1]).clip(1,len(lt)-1)
 j-=abs(lt[j-1]-c[:,1])<abs(lt[j]-c[:,1])
 gap=abs(lt[j]-c[:,1])
 a=c[:,4:].reshape(-1,16,7); slots=a[0,:,0].astype(int)
 assert np.all(a[:,:,0]==slots)
 q,dq,_,tau,*_=l[ix[j],13:].reshape(-1,16,6)[:,slots].transpose(2,0,1)
 pd=a[:,:,4]*(a[:,:,2]-q)+a[:,:,5]*(a[:,:,3]-dq)+a[:,:,6]
 ok=gap<=2
 result[s]={'frames':len(c),'within_2_tick_units':int(ok.sum()),'exact_tick':int((gap==0).sum()),'leg_mae_nm':float(abs(pd-tau)[ok,:12].mean()),'wheel_mae_nm':float(abs(pd-tau)[ok,12:].mean()),'leg_pd_peak_nm':float(abs(pd[ok,:12]).max())}
(p/'lowcmd-audit.json').write_text(json.dumps(result,indent=2))
print(result)
