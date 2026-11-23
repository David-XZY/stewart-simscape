"""One-time, outcome-independent migration of pre-freeze CV alias records."""
import hashlib
import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent
TEACHER = ROOT / 'artifacts' / 'teacher'
ARCHIVE = ROOT / 'history' / 'ch5-v003-cv-alias'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def move(path, folder):
    assert path.resolve().is_relative_to(ROOT.resolve())
    destination = folder / path.name
    assert destination.resolve().is_relative_to(ARCHIVE.resolve())
    assert not destination.exists(), f'Preserve existing audit: {destination}'
    folder.mkdir(parents=True, exist_ok=True)
    shutil.move(str(path), str(destination))


def main():
    assert not ARCHIVE.exists(), 'This migration has already run.'
    moving = set()
    for path in TEACHER.glob('*_force_*.json'):
        raw = json.loads(path.read_text('utf-8'))
        if raw['task']['family'] == 4:
            moving.add(path.with_suffix('.mat').name)
    for path in TEACHER.glob('*.attempts.jsonl'):
        for line in path.read_text('utf-8').splitlines():
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue  # Interrupted append remains preserved in its raw file.
            if value.get('task', {}).get('family') == 4:
                moving.add(path.name.removesuffix('.attempts.jsonl'))
    excluded = []
    for name in sorted(moving):
        mat = TEACHER / name
        paths = set(TEACHER.glob(name + '*')) | {mat.with_suffix('.json')}
        for path in sorted(paths):
            if path.exists():
                excluded.append({'path': path.name, 'sha256': digest(path) if path.is_file() else None})
                move(path, ARCHIVE / 'teacher')
    parents = {}
    admitted = []
    snapshots = {digest(ROOT / name): name for name in ['teacher_precheck_parent_ihsid.txt', 'teacher_cv_parent_ihsid.txt']}
    for path in sorted(TEACHER.glob('*_force_baseline_0.json')):
        raw = json.loads(path.read_text('utf-8'))
        assert raw['status'] in ['completed', 'simulation_failed']
        assert raw['task']['family'] != 4
        assert all(value == 0 for row in raw['task']['motion_velocity'] for value in row)
        source = raw['source_identity'];ihsid = source['files']['ch5Ihsid_m']
        assert ihsid in snapshots
        for field, checksum in source['files'].items():
            if field == 'ch5Ihsid_m':
                continue
            stem, suffix = field.rsplit('_', 1)
            assert digest(ROOT / f'{stem}.{suffix}') == checksum, f'Unrelated source changed: {field}'
        parents[source['baseline_source_sha256']] = {'baseline_source_sha256': source['baseline_source_sha256'],
            'ihsid_sha256': ihsid, 'source_file': snapshots[ihsid]}
        mat = path.with_suffix('.mat')
        admitted.append({'file': mat.name, 'sha256': digest(mat), 'baseline_source_sha256': source['baseline_source_sha256']})
    registry = ROOT / 'teacher_source_admissions.json'
    ARCHIVE.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(registry, ARCHIVE / registry.name)
    admission = {'schema': 2, 'version': 'ch5-v003-development',
        'reason': 'All completed stationary-obstacle teacher runs are admitted independent of outcome. CV aliasing affects only nonzero obstacle velocity; all moving-obstacle records are excluded and recollected. Fixed-boundary precheck provenance remains explicit.',
        'new_ihsid_sha256': digest(ROOT / 'ch5Ihsid.m'), 'parents': list(parents.values()), 'runs': admitted}
    registry.write_text(json.dumps(admission, ensure_ascii=False, indent=2), 'utf-8')
    for path in (ROOT / 'artifacts' / 'validation').glob('*_force_*.mat*'):
        move(path, ARCHIVE / 'validation')
    for path in (ROOT / 'artifacts' / 'validation').glob('*_force_*.json'):
        move(path, ARCHIVE / 'validation')
    for name in ['interface.mat', 'interface.json']:
        path = ROOT / 'artifacts' / 'validation' / name
        if path.exists():
            move(path, ARCHIVE / 'validation')
    audit = {'excluded_moving_files': excluded, 'admitted_stationary_runs': len(admitted),
             'moving_episode_identities': sorted(moving), 'formal_runs_started': 0,
             'reason': admission['reason']}
    (ARCHIVE / 'audit.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2), 'utf-8')
    print(json.dumps({'admitted': len(admitted), 'moving_identities_excluded': len(moving)}))


if __name__ == '__main__':
    main()
