"""Read-only snapshot of study records; excludes large trajectory MAT files."""
import argparse
from pathlib import Path
import zipfile

p = argparse.ArgumentParser()
p.add_argument('run', nargs='?', default='frozen_v1')
p.add_argument('--full', action='store_true', help='Include completed trajectory MAT files and resume data.')
p.add_argument('--references-only', action='store_true', help='Copy only the separate checkpointed reference repeat.')
a = p.parse_args()
root = Path(__file__).resolve().parents[1] / 'results' / 'study' / a.run
name='full_snapshot.zip' if a.full else 'snapshot.zip'
if a.references_only: name='reference_snapshot.zip'
with zipfile.ZipFile(root / name, 'w', zipfile.ZIP_DEFLATED) as archive:
    files = list(root.glob('*.json')) + list(root.glob('*.log')) + list(root.glob('*.err'))
    files += list(root.glob('jobs/*/result.json')) + list(root.glob('jobs/*/online_progress.json'))
    files += list(root.glob('exports/*.json'))
    files += list(root.glob('environment_recovery_*/*.json'))
    files += list(root.glob('reference_checkpointed_v1/**/*.json')) + list(root.glob('reference_checkpointed_v1/**/*.log'))
    if a.full:
        files += list(root.glob('*.mat'))
        for result in root.glob('jobs/*/result.json'):
            files += list(result.parent.glob('*.mat'))
        files += [f for f in root.glob('nominal_reuse/*.mat') if not f.name.endswith('.tmp.mat')]
        files += [f for f in root.glob('jobs/*/interrupted_*/*') if f.is_file()]
        files += list(root.glob('reference_checkpointed_v1/**/*.mat'))
    if a.references_only:
        files=[f for f in (root/'reference_checkpointed_v1').rglob('*') if f.is_file() and f.suffix in ('.json','.mat','.log')]
    for f in files:
        archive.write(f, f.relative_to(root))
print(root / name)
