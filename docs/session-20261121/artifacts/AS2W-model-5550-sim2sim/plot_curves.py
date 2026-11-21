from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
for terrain in ('flat','slope'):
 old=np.load('isaac-'+terrain+'.npz')
 new=np.load(terrain+'.npz')
 for mode in ('stand','forward','reverse','left','right','yaw_left','yaw_right','mixed'):
  i=np.flatnonzero(new['labels']==mode)[0]
  t=.02*(1+np.arange(len(new['q'])))
  fall=np.flatnonzero(new['fall'][:,i])
  end=fall[0]+1 if len(fall) else len(t)
  fig,ax=plt.subplots(4,4,figsize=(15,9),sharex=True)
  for w in range(4):
   for j in range(3):
    q=w*3+j;a=ax[w,j]
    a.plot(t,np.rad2deg(old['q'][:,i,q]),label='Isaac actual')
    a.plot(t[:end],np.rad2deg(new['q'][:end,i,q]),label='MuJoCo actual')
    a.plot(t[:end],np.rad2deg(new['target_trace'][:end,i,1,q]),':',label='MuJoCo target')
    a.set_title(str(new['joints'][q]));a.set_ylabel('deg')
   a=ax[w,3];a.plot(t,old['normal_load'][:,i,w]);a.plot(t[:end],new['normal_load'][:end,i,w]);a.set_title(str(new['joints'][12+w]));a.set_ylabel('N')
  for a in ax.flat:
   a.grid(alpha=.22)
   if len(fall):a.axvline(t[fall[0]],color='r',alpha=.45)
  ax[0,0].legend(fontsize=7)
  for a in ax[-1]:a.set_xlabel('Time (s)')
  fig.suptitle(f'model_5550 | {terrain} | {mode} | trial 0 | red = MuJoCo fall threshold')
  fig.tight_layout()
  fig.savefig(f'{terrain}-{mode}-joint-support.png',dpi=130)
  plt.close(fig)
