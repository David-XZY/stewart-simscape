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
    result.extend(p for p in (ROOT / 'chapter5').glob('*') if p.is_file() and p.suffix in {'.m', '.py', '.json', '.ps1', '.txt', '.md'})
    result.extend((ART / 'models').glob('seed_*.mat'))
    result.extend((ART / 'models').glob('seed_*.json'))
    result.extend((ART / 'models').glob('seed_*.pt'))
    result.extend((ART / 'models').glob('seed_*_training.csv'))
    result.extend((ART / 'teacher').glob('*windows.h5'))
    result.extend((ART / 'teacher').glob('*windows_provenance.mat'))
    result.extend((ART / 'teacher').glob('dataset_manifest.json'))
    result.extend((ART / 'validation').glob('environment.json'))
    result.append(ART / 'validation' / 'physical_parameters.json')
    for name in ['interface.mat', 'numerics.mat', 'derivative_map.mat', 'native_collision.mat', 'moving_prediction.mat', 'control_api.mat', 'inference.mat', 'network_api_probe.mat', 'development_audit.mat']:
        result.append(ART / 'validation' / name)
    result.extend((ROOT / 'lib').rglob('*.dll'))
    result.extend((ROOT / 'lib').rglob('*.mexw64'))
    return sorted(set(result))


def freeze():
    assert not MANIFEST.exists(), 'A frozen experiment is immutable. Use a new experiment version.'
    config = json.loads((ROOT / 'chapter5' / 'config.json').read_text('utf-8'))
    assert not (ART / 'tasks' / 'test.mat').exists(), 'Holdout tasks must be generated after the freeze.'
    for seed in config['training_seeds']:
        assert (ART / 'models' / f'seed_{seed}.mat').exists()
    physical = json.loads((ART / 'validation' / 'physical_parameters.json').read_text('utf-8'))
    assert physical['passed'] and physical['version'] == config['version'] and physical['Ts'] == config['Ts']
    for check in ['interface.mat', 'numerics.mat', 'derivative_map.mat', 'native_collision.mat', 'moving_prediction.mat', 'control_api.mat', 'inference.mat']:
        value = scipy.io.loadmat(ART / 'validation' / check, simplify_cells=True)
        assert value['report']['passed']
    api = scipy.io.loadmat(ART / 'validation' / 'control_api.mat', simplify_cells=True)['report']
    for name, checksum in api['code_hashes'].items():
        filename = name.rsplit('_', 1)
        assert digest(ROOT / 'chapter5' / f'{filename[0]}.{filename[1]}') == checksum, 'The API validation must match current source/configuration.'
    development = list((ART / 'development').glob('*.json'))
    assert len(development) == config['development_tasks'] * 12
    assert all(json.loads(p.read_text('utf-8'))['status'] in ['completed', 'simulation_failed'] for p in development)
    for p in development:
        record = json.loads(p.read_text('utf-8'))
        assert record['version'] == config['version']
        assert p.with_suffix('.mat').exists()
        assert record['status'] != 'simulation_failed' or record.get('error'), 'A failed simulation needs its explicit diagnostic'
    # Recompute this audit from the current code/model identities and all
    # raw development records. It is executed only before a new freeze;
    # verification/resume never rewrites a frozen prerequisite report.
    subprocess.run(['D:/MATLAB/R2025b/bin/matlab.exe', '-singleCompThread', '-batch',
                    "addpath('chapter5');ch5AuditDevelopment;", '-logfile',
                    str(ROOT / 'chapter5' / 'development_audit.log')], cwd=ROOT,
                   check=True, creationflags=subprocess.CREATE_NO_WINDOW)
    audit = scipy.io.loadmat(ART / 'validation' / 'development_audit.mat', simplify_cells=True)['report']
    assert audit['passed'] and audit['version'] == config['version'] and audit['runs'] == config['development_tasks'] * 12
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
