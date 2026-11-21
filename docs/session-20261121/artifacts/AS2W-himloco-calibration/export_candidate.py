from pathlib import Path
import os, json, shutil
p=Path(__file__).resolve().parent
os.environ.update(ALT_HIP='1',EXCLUDE_FRONT_THIGHS='1',BASE_SCALE='1',BASE_DX='0')
exec((p/'open_loop.py').read_text().split('results={}')[0])
out=p/'asset-candidate-v1'
out.mkdir(exist_ok=True)
for mesh in x.findall('asset/mesh'):
 src=Path(mesh.get('file'))
 dst=out/'meshes'/src.name
 dst.parent.mkdir(exist_ok=True)
 shutil.copy2(src,dst)
 mesh.set('file','meshes/'+src.name)
act=ET.parse('AS2W-github-publish/review/u3220/model/as2w_flat.xml').getroot().find('actuator')
assert act is not None
x.append(act)
asset=out/'as2w.xml'
ET.ElementTree(x).write(asset,encoding='utf-8',xml_declaration=True)
loaded=mj.MjModel.from_xml_path(str(asset))
assert loaded.nu==16 and loaded.nq==m.nq and loaded.nexclude==m.nexclude
assert np.allclose(loaded.body_mass,m.body_mass)
(out/'contract.json').write_text(json.dumps({'scope':'MuJoCo offline candidate; not an Isaac Lab training asset','dds_to_mujoco_slots':[3,4,5,0,1,2,9,10,11,6,7,8,13,12,15,14],'dds_to_mujoco_sign':sign.tolist(),'mass_kg':float(loaded.body_mass.sum()),'timestep_seconds':float(loaded.opt.timestep),'excluded_body_pairs':[['base_link','FL_thigh'],['base_link','FR_thigh']],'hardware_mapping_changed':False},indent=2))
archive=shutil.make_archive(str(out),'zip',root_dir=out.parent,base_dir=out.name)
print(loaded.nu,loaded.body_mass.sum(),loaded.nexclude,archive)
