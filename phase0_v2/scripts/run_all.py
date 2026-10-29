"""One command: preserved v1 regression, dual pilot, physical checks and analysis."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

REPO=Path(__file__).resolve().parents[2]
ROOT=REPO/'phase0_v2'


def provenance(output,stages):
    sources=sorted(ROOT.rglob('*.py'))+list((ROOT/'models').glob('*.xml'))
    sources+=[REPO/'phase0_tactile/scripts/run_experiment.py',REPO/'phase0_tactile/scripts/validate_physics.py',
              REPO/'phase0_tactile/models/tactile_pilot.xml',REPO/'phase0_tactile/requirements.txt']
    payload={'recorded_utc':datetime.now(timezone.utc).isoformat(),'stages':stages,
             'source_sha256':{str(p.relative_to(REPO)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(sources)},
             'artifact_sha256':{str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.rglob('*')) if p.is_file() and p.name!='provenance.json'}}
    (output/'provenance.json').write_text(json.dumps(payload,indent=2)+'\n')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=ROOT/'results/reproduction');ap.add_argument('--render',action='store_true');a=ap.parse_args();out=a.output.resolve()
    if out.exists():ap.error('choose a fresh output directory')
    stages=[]
    for module,dest,extra in [('regress_v1',out/'v1_regression',[]),('run_pilot',out/'dual',['--render'] if a.render else []),('validate_pilot',out/'dual',[]),('analyze_pilot',out/'dual',[])]:
        start=time.perf_counter()
        subprocess.run([sys.executable,'-m','phase0_v2.scripts.'+module,'--output',str(dest),*extra],cwd=REPO,check=True)
        stages.append(dict(stage=module,elapsed_seconds=time.perf_counter()-start,passed=True))
    provenance(out,stages)
    print('Complete:',out)

if __name__=='__main__':main()
