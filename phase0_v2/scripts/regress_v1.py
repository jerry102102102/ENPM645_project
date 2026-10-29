"""Run representative v1 A/B/C and its checks without modifying the reference tree."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

REPO=Path(__file__).resolve().parents[2]
V1=REPO/'phase0_tactile'


def protected_hashes():
    files=[p for p in V1.iterdir() if p.is_file()]
    files+=list((V1/'models').glob('*.xml'))+list((V1/'scripts').glob('*.py'))
    files += [p for p in (V1/'results/pilot').rglob('*') if p.is_file()]
    return {str(p.relative_to(REPO)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}


def verify(output,before):
    assert before==protected_hashes(),'v1 reference changed'
    arrays=0
    for scenario in 'ABC':
        for name in ('observations.npz','privileged.npz','diagnostics.npz'):
            rel=Path('data')/(scenario+'_000')/name
            with np.load(V1/'results/pilot'/rel) as a,np.load(output/rel) as b:
                assert a.files==b.files
                for key in a.files:
                    assert np.array_equal(a[key],b[key]),str(rel)+':'+key
                    arrays+=1
    controls=json.loads((output/'physics_controls.json').read_text())
    assert controls['passed']
    result=dict(passed=True,episodes=3,arrays_exactly_equal=arrays,original_files_unchanged=True,protected_sha256=before)
    (output/'regression.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():ap.error('use a new directory')
    before=protected_hashes()
    subprocess.run([sys.executable,str(V1/'scripts/run_experiment.py'),'--episodes','1','--output',str(a.output)],check=True)
    subprocess.run([sys.executable,str(V1/'scripts/validate_physics.py'),'--output',str(a.output)],check=True)
    r=verify(a.output,before);print('v1 regression:',r['passed'],r['arrays_exactly_equal'],'arrays equal')

if __name__=='__main__':main()
