from pathlib import Path
import json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(sys.argv[1])
t=(np.arange(300)+1)*.02
for scenario in ('usd_min','usd_max','rear_load_min'):
    paths=[root/'pressure'/f'{scenario}-{a}' for a in 'AB']
    if not all((p/'metrics.json').exists() for p in paths): continue
    contracts=[json.loads((p/'fixture-contract.json').read_text()) for p in paths]
    assert contracts[0]==contracts[1], scenario
    data=[dict(np.load(p/'flat.npz')) for p in paths]
    a,b=data
    for k in ('cmd','labels'): np.testing.assert_array_equal(a[k],b[k])
    assert not a['fall'].any() and not b['fall'].any(), scenario
    fig,axes=plt.subplots(5,2,figsize=(13,12),sharex=True,layout='constrained')
    for col,label in enumerate(('forward','reverse')):
        for z,arm,color in zip(data,'AB',['#3366aa','#d34a2c']):
            mask=z['labels']==label
            x,y,zz,w=np.moveaxis(z['pose'][...,3:7],-1,0)
            pitch=np.degrees(np.arcsin(np.clip(2*(w*y-zz*x),-1,1)))
            q=np.degrees(z['q'][...,:12]).reshape(300,-1,4,3)
            tau=np.abs(z['leg_torque_nm']).reshape(q.shape)
            f=z['wheel_ground_normal_n']
            for row,curves in enumerate(((pitch[:,mask],),(z['vel'][...,0][:,mask],),
                (q[:,mask][:,:,:2,1].mean(2),q[:,mask][:,:,2:,1].mean(2)),
                (tau[:,mask][:,:,:2,0].mean(2),tau[:,mask][:,:,2:,0].mean(2)),
                (f[:,mask][:,:,:2].sum(2),f[:,mask][:,:,2:].sum(2)))):
                ax=axes[row,col]
                for j,v in enumerate(curves):
                    ax.plot(t,np.median(v,axis=1),color=color,ls='-' if j==0 else '--',
                        label=arm if len(curves)==1 else f'{arm} '+('front' if j==0 else 'rear'))
                    if row==0: ax.fill_between(t,*np.percentile(v,[10,90],axis=1),color=color,alpha=.12)
        axes[1,col].plot(t,a['cmd'][...,0][:,a['labels']==label].mean(1),':',color='black',label='command')
        axes[0,col].set_title(label)
        axes[-1,col].set_xlabel('Time (s)')
        for row,title in enumerate(('Pitch (deg)','Forward speed (m/s)','Thigh angle (deg)','Hip |torque| (Nm)','Axle normal load (N)')):
            ax=axes[row,col]
            ax.set_ylabel(title)
            ax.axvline(2,color='.65',lw=.8)
            ax.axvline(5,color='.65',lw=.8)
            ax.grid(alpha=.18)
            if col==0: ax.legend(loc='upper right',fontsize=8,ncol=2)
        for th in (-4,4): axes[0,col].axhline(th,color='.35',lw=.7,ls=':')
    fig.suptitle(f'HIMLoco 10436 | {scenario} | A baseline, B persistent pitch')
    fig.savefig(root/f'{scenario}.png',dpi=135)
    plt.close(fig)
    initial=contracts[0]['initial_pitch_deg']
    one=next(iter(contracts[0]['fixtures'].values()))
    speed=np.unique(a['cmd'][:,:,0])
    print(scenario, 'initial pitch',min(initial),max(initial),'speed',speed,'fixture keys',list(one),flush=True)
