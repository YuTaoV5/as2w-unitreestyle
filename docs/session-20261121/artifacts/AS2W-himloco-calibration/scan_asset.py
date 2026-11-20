from pathlib import Path
import subprocess,os,json
p=Path(__file__).parent
out={}
for scale,dx in [(1,0),(.9,0),(1.1,0),(1,-.02),(1,.02),(1.1,-.02),(.9,.02)]:
 env=dict(os.environ,BASE_SCALE=str(scale),BASE_DX=str(dx),SCAN_ONLY='1')
 subprocess.run(['python',str(p/'open_loop.py')],env=env,check=True)
 out[f'{scale},{dx}']=json.loads((p/'scan-case.json').read_text())
(p/'asset-scan.json').write_text(json.dumps(out,indent=2))
