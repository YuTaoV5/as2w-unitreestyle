"""Read-only field schema audit; never connects to the robot."""
import csv,json,math
from pathlib import Path
import numpy as np
ROOT=Path(r'X:/Items/as2w/himloco_field_20261119')
OUT=Path(__file__).resolve().parent
report={}
for session in ['vx015_walked','vx010_X_abort_handoff']:
    rows=[]
    for r in csv.reader((ROOT/session/'debug_policy.csv').open()):
        if not r or r[0]!='DEBUG_POLICY': continue
        n=int(r[57]); assert n==360
        obs=np.array(r[58:58+n],float)
        k=58+n; ns=int(r[k]); k+=1+ns
        raw=np.array(r[k:k+16],float); k+=32
        req=np.array(r[k:k+64],float).reshape(16,4); k+=64
        app=np.array(r[k:k+64],float).reshape(16,4); k+=64
        assert k==len(r),(k,len(r))
        q=np.array(r[25:41],float); dq=np.array(r[41:57],float)
        quat=np.array(r[18:22],float); w,x,y,z=quat
        pitch=math.degrees(math.asin(np.clip(2*(w*y-z*x),-1,1)))
        pd=app[:,2]*(app[:,0]-q)+app[:,3]*(app[:,1]-dq)
        rows.append(dict(phase=r[2],obs=obs,req=req,app=app,q=q,dq=dq,pd=pd,pitch=pitch))
    result={}
    for phase in sorted(set(r['phase'] for r in rows)):
        ix=[i for i,r in enumerate(rows) if r['phase']==phase]
        pds=np.array([rows[i]['pd'] for i in ix])
        actual_errors=[]; request_errors=[]
        for i in ix:
            if i==0: continue
            def feedback(t):
                return np.r_[(t[:12,0]-np.tile([0,.8,-1.6],4))/.25,t[12:,1]/12]
            actual_errors.append(np.max(abs(rows[i]['obs'][41:57]-feedback(rows[i-1]['app']))))
            request_errors.append(np.max(abs(rows[i]['obs'][41:57]-feedback(rows[i-1]['req']))))
        result[phase]={'frames':len(ix),'pitch_first_last':[rows[ix[0]]['pitch'],rows[ix[-1]]['pitch']],
            'feedback_vs_previous_applied_max':max(actual_errors,default=0),
            'feedback_vs_previous_requested_max':max(request_errors,default=0),
            'pd_mean_abs':np.mean(abs(pds),axis=0).tolist(),'pd_max_abs':np.max(abs(pds),axis=0).tolist(),
            'initial_q':rows[ix[0]]['q'].tolist()}
    report[session]=result
(OUT/'field-audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
