"""Freeze code/models/protocol before drawing test tasks; never edit a freeze."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path
import scipy.io

ROOT = Path(__file__).resolve().parent.parent
ART = ROOT / 'chapter5' / 'artifacts'
MANIFEST = ART / 'freeze.json'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def files():
    tracked = subprocess.check_output(['git', 'ls-files', '-z'], cwd=ROOT).decode('utf-8').split('\0')
    result = [ROOT / name for name in tracked if name and (ROOT / name).is_file()]
    result.extend(p for p in (ROOT / 'chapter5').glob('*') if p.is_file() and p.suffix in {'.m', '.py', '.json', '.ps1', '.txt'})
    result.extend((ART / 'models').glob('seed_*.mat'))
    result.extend((ART / 'models').glob('seed_*.json'))
    result.extend((ART / 'models').glob('seed_*.pt'))
    result.extend((ART / 'models').glob('seed_*_training.csv'))
    result.extend((ART / 'teacher').glob('*windows.h5'))
    result.extend((ART / 'teacher').glob('*windows_provenance.mat'))
    result.extend((ART / 'teacher').glob('dataset_manifest.json'))
    result.extend((ART / 'validation').glob('environment.json'))
    result.extend((ROOT / 'lib').rglob('*.dll'))
    result.extend((ROOT / 'lib').rglob('*.mexw64'))
    return sorted(set(result))


def freeze():
    assert not MANIFEST.exists(), 'A frozen experiment is immutable. Use a new experiment version.'
    config = json.loads((ROOT / 'chapter5' / 'config.json').read_text('utf-8'))
    assert not (ART / 'tasks' / 'test.mat').exists(), 'Holdout tasks must be generated after the freeze.'
    for seed in config['training_seeds']:
        assert (ART / 'models' / f'seed_{seed}.mat').exists()
    for check in ['interface.mat', 'numerics.mat', 'inference.mat']:
        value = scipy.io.loadmat(ART / 'validation' / check, simplify_cells=True)
        assert value['report']['passed']
    development = list((ART / 'development').glob('*.json'))
    assert len(development) == config['development_tasks'] * 12
    assert all(json.loads(p.read_text('utf-8'))['status'] == 'completed' for p in development)
    result = {'version': config['version'], 'baseline_commit': config['baseline_commit'],
              'artifacts': {str(p.relative_to(ROOT)).replace('\\', '/'): digest(p) for p in files()}}
    MANIFEST.write_text(json.dumps(result, ensure_ascii=False, indent=2), 'utf-8')


def verify():
    frozen = json.loads(MANIFEST.read_text('utf-8'))
    differences = [name for name, value in frozen['artifacts'].items() if not (ROOT / name).exists() or digest(ROOT / name) != value]
    assert not differences, f'Frozen files changed: {differences}'
    print(f"Verified {len(frozen['artifacts'])} frozen artifact hashes.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    verify() if args.verify else freeze()
