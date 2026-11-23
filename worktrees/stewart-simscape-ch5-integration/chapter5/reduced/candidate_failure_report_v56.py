"""Recompute constraint and paired denoising diagnosis from preserved artifacts."""
from pathlib import Path
import csv
import hashlib
import json
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'chapter5/artifacts/reduced_probe'
OUT = BASE / 'denoise_mechanism_v56'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text('utf8'))


def write_csv(name, rows):
    with (OUT / name).open('w', encoding='utf8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    mechanical = read(BASE / 'candidate_failure_details_v55/summary.json')
    teacher = read(BASE / 'teacher_candidate_audit_v39/summary.json')
    neural = read(OUT / 'neural_summary.json')
    paired = read(OUT / 'physical_summary.json')
    assert all(a['completed'] and not a.get('error') for a in [mechanical, neural, paired])
    assert len(mechanical['rows']) == 24 and len(paired['rows']) == 1440
    assert sha(BASE / 'teacher_candidate_audit_v39/audit.mat') == mechanical['source_sha256'] == neural['source_sha256']
    assert sha(OUT / 'candidates.mat') == neural['candidates_sha256']
    for model in neural['models']:
        assert sha(ROOT / f"chapter5/artifacts/models/seed_{model['seed']}.mat") == model['weight_sha256']
    production = hashlib.sha256()
    for path in sorted((ROOT / 'chapter5').iterdir()):
        if path.suffix in {'.m', '.py', '.json'}:
            production.update(path.name.encode())
            production.update(path.read_bytes())
    assert production.hexdigest() == 'a0735e0e37317afcf4107359797e9fbab71fcb886584beac318b42edbcb050d6'
    flat = []
    for row in mechanical['rows']:
        for sample in row['generated']:
            entry = {'mode': row['mode'], 'window': row['window_index'], 'seed': sample['seed'], 'candidate': sample['candidate'],
                     'finite': sample['old_check']['finite'], 'first_violation_seconds': sample['mechanics']['first_mechanical_violation_seconds'],
                     'increment_error_rms_N': sample['increment_rms_error_N'], 'force_error_rms_N': sample['force_rms_error_N']}
            for name in ['length', 'speed', 'acceleration', 'singularity_squared', 'actual_force']:
                entry[f'{name}_violated'] = sample['mechanics'][name]['violated']
            flat.append(entry)
    write_csv('original_constraints.csv', flat)
    write_csv('forward_noise.csv', neural['forward_noise'])
    summaries = []
    for variant in ['original20', 'identity99_20', 'dense100']:
        rows = [a for a in paired['rows'] if a['variant'] == variant]
        assert len(rows) == 480
        summaries.append({'variant': variant, 'samples': len(rows),
            'finite': sum(a['check']['finite'] for a in rows),
            'mechanical_and_collision_valid': sum(a['check']['internal_check'].get('accepted', False) for a in rows),
            'all_inequality_valid': sum(a['check']['inequality_violation'] is not None and a['check']['inequality_violation'] <= 1e-6 for a in rows),
            'terminal_valid': sum(a['check']['equality_tolerance_ratio'] is not None and a['check']['equality_tolerance_ratio'] <= 1 for a in rows),
            'full_valid': sum(a['check']['accepted'] for a in rows),
            'increment_error_median_N': float(np.median([a['increment_rms_error_N'] for a in rows])),
            'force_error_median_N': float(np.median([a['force_rms_error_N'] for a in rows]))})
    write_csv('paired_denoising.csv', summaries)
    finite = [a for a in flat if a['finite']]
    summary = {'status': 'development diagnosis only', 'production_source_sha256': production.hexdigest(),
               'original_samples': len(flat), 'original_finite': len(finite),
               'finite_mechanical_violations': {name: sum(a[f'{name}_violated'] for a in finite)
                   for name in ['length', 'speed', 'acceleration', 'singularity_squared', 'actual_force']},
               'first_violation_median_seconds': float(np.median([a['first_violation_seconds'] for a in finite])),
               'teacher_windows': len(teacher['rows']),
               'teacher_mechanical_valid': sum(a['teacher_raw_check']['internal_check']['accepted'] for a in teacher['rows']),
               'teacher_raw_terminal_valid': sum(a['teacher_raw_check']['equality_tolerance_ratio'] <= 1 for a in teacher['rows']),
               'teacher_after_terminal_repair_full_valid': sum(a['teacher_repaired_check']['accepted'] for a in teacher['rows']),
               'paired_sampling': summaries,
               'interpretation': 'identity99_20 changes only the first epsilon prediction to its noisy input. It is a diagnostic ablation, not an accepted online controller. dense100 changes only the sampling-step count. Common Gaussian noise is fixed within each seed/window/candidate. Teacher RMS error is a diagnostic, not a feasibility criterion.',
               'boundary': '1440 entries are paired prediction checks, not formal Simscape runs; formal campaign remains 0/1440.',
               'references': ['https://arxiv.org/html/2305.08891v4', 'https://proceedings.mlr.press/v283/romer25a.html']}
    source_paths = [ROOT / 'chapter5/reduced' / a for a in [
        'ch5SpeedCandidateFailureDetailsV55.m', 'candidate_denoise_mechanism_v56.py',
        'ch5SpeedDenoiseMechanismAuditV56.m', 'candidate_failure_report_v56.py']]
    artifacts = [BASE / 'candidate_failure_details_v55/audit.mat', BASE / 'candidate_failure_details_v55/summary.json',
                 OUT / 'candidates.mat', OUT / 'neural_summary.json', OUT / 'physical_audit.mat', OUT / 'physical_summary.json']
    summary['manifest'] = {str(path.relative_to(ROOT)): {'sha256': sha(path), 'bytes': path.stat().st_size}
                           for path in source_paths + artifacts}
    with zipfile.ZipFile(OUT / 'diagnostic_sources.zip', 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for path in source_paths:
            archive.write(path, str(path.relative_to(ROOT)))
    (OUT / 'aggregate_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf8')
    print(json.dumps({key: value for key, value in summary.items() if key != 'manifest'}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
