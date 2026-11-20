"""Sample the direction of minimum support of each tire's radial envelope."""
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull
import trimesh

root=Path(__file__).resolve().parents[1]
tilt=np.linspace(-1.,1.,513)
directions=np.stack((tilt,-np.sqrt(np.maximum(0.,1-tilt**2))),axis=1)
profiles=[]
for wheel in ("FL","FR","RL","RR"):
    mesh=root/"assets/as2w/meshes"/f"{wheel}_foot.STL"
    v=trimesh.load_mesh(mesh,process=True).vertices
    profile=np.stack((v[:,1],np.linalg.norm(v[:,[0,2]],axis=1)),axis=1)
    envelope=profile[ConvexHull(profile).vertices]
    profiles.append(envelope[(directions@envelope.T).argmin(1)])
np.save(Path(__file__).with_name("tire_profile.npy"),np.array(profiles,dtype=np.float32))
