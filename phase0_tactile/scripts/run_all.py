"""Run the complete 30-episode headless pilot into a fresh output folder."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, default=ROOT/'results/reproduction')
    p.add_argument('--render', action='store_true', help='Optional scene snapshots; no GUI')
    a = p.parse_args()
    if a.output.exists():
        p.error('Output already exists; choose a fresh path to preserve previous results')
    for script in ('run_experiment.py', 'validate_physics.py', 'analyze_results.py'):
        command = [sys.executable, str(ROOT/'scripts'/script), '--output', str(a.output)]
        if script == 'run_experiment.py' and a.render:
            command.append('--render')
        subprocess.run(command, check=True)
    sources = sorted((ROOT/'scripts').glob('*.py')) + [ROOT/'models/tactile_pilot.xml', ROOT/'requirements.txt']
    provenance = {'completed_utc': datetime.now(timezone.utc).isoformat(),
                  'source_sha256': {str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},
                  'artifact_sha256': {str(f.relative_to(a.output)): hashlib.sha256(f.read_bytes()).hexdigest()
                                      for f in sorted(a.output.rglob('*')) if f.is_file()}}
    (a.output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    print(f'Complete: {a.output.resolve()}')


if __name__ == '__main__':
    main()
