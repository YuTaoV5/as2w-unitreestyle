from pathlib import Path
import shutil,xml.etree.ElementTree as ET,json
p=Path('AS2W-unitree-flat-v2-2000')
asset=p/'assets';asset.mkdir(exist_ok=True)
src=Path('AS2W-github-publish/unified_dr/simulation/assets/load-v0')
shutil.copytree(src/'meshes',asset/'meshes',dirs_exist_ok=True)
tree=ET.parse(src/'as2w-load-v0.urdf')
origin=tree.find("link[@name='base_link']/inertial/origin")
xyz=list(map(float,origin.get('xyz').split()));xyz[0]+=.02
origin.set('xyz',' '.join(map(str,xyz)))
tree.write(asset/'as2w-t2-v2.urdf',encoding='utf-8',xml_declaration=True)
f=p/'environment.py';s=f.read_text();s=s.replace('source = ROOT / "assets/as2w/as2w.urdf"','source = Path(__file__).parent / "assets/as2w-t2-v2.urdf"')
f.write_text(s)
f=p/'run.py';s=f.read_text().replace('args=parser.parse_args()','parser.add_argument("--terrain", choices=("flat",), default="flat")\nargs=parser.parse_args()');f.write_text(s)
f=p/'task.py';s=f.read_text().replace('    env=Task(cfg)','''    cfg.scene.terrain.terrain_generator.curriculum=False
    env=Task(cfg)
    from v2_collision import apply_collision_exclusions
    apply_collision_exclusions(env.scene.stage)
''');s=s.replace('apply_collision_exclusions(env.scene.stage)','apply_collision_exclusions()');f.write_text(s)
