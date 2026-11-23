"""Verify immutable v003 teacher-trained weights after a numerical control repair.

Only listed execution/solver settings may differ from the byte-preserved
training configuration. Labels, normalization, seeds and training choices do
not change. No checkpoint is rewritten or selected using development results.
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RUNTIME_FIELDS = {'local_workers', 'control_revision', 'projection_qp_max_iterations',
                  'final_projection_iterations'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_pretrained(seed, configuration):
    original_file = ROOT / 'training_config_v003.json'
    original = json.loads(original_file.read_text('utf-8'))
    old_semantics = {k: v for k, v in original.items() if k not in RUNTIME_FIELDS}
    new_semantics = {k: v for k, v in configuration.items() if k not in RUNTIME_FIELDS}
    assert old_semantics == new_semantics, 'Training/physical/task configuration changed; retraining is required.'
    models = ROOT / 'artifacts' / 'models'
    metadata_file = models / f'seed_{seed}.json'
    metadata = json.loads(metadata_file.read_text('utf-8'))
    assert metadata['completed'] and metadata['version'] == original['version']
    assert metadata['seed'] == seed and seed in original['training_seeds']
    assert metadata['configuration_sha256'] == digest(original_file)
    assert metadata['trainer_sha256'] == digest(ROOT / 'training_source_v003.py.txt')
    manifest = ROOT / 'artifacts' / 'teacher' / 'dataset_manifest.json'
    assert metadata['dataset_manifest_sha256'] == digest(manifest)
    assert metadata['checkpoint_sha256'] == digest(models / f'seed_{seed}.pt')
    assert metadata['matlab_sha256'] == digest(models / f'seed_{seed}.mat')
    dataset = json.loads(manifest.read_text('utf-8'))
    assert len(dataset['raw_runs']) == 400
    for split in ['train', 'validation']:
        assert dataset[f'{split}_sha256'] == digest(manifest.parent / f'{split}_windows.h5')
    return {'seed': seed, 'metadata_sha256': digest(metadata_file),
            'matlab_sha256': metadata['matlab_sha256'], 'checkpoint_sha256': metadata['checkpoint_sha256'],
            'training_configuration_sha256': digest(original_file),
            'runtime_configuration_sha256': digest(ROOT / 'config.json')}


def main():
    configuration = json.loads((ROOT / 'config.json').read_text('utf-8'))
    records = [verify_pretrained(seed, configuration) for seed in configuration['training_seeds']]
    report = {'passed': True, 'control_revision': configuration['control_revision'],
              'allowed_runtime_fields': sorted(RUNTIME_FIELDS), 'models': records}
    (ROOT / 'artifacts' / 'validation' / 'model_compatibility.json').write_text(json.dumps(report, indent=2), 'utf-8')
    print('Verified all five immutable teacher-trained models and unchanged training/physical/task semantics.')


if __name__ == '__main__':
    main()
