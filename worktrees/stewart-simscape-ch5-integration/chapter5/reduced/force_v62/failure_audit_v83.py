"""Trace preserved failures without repairing labels or screening tasks."""
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'chapter5/artifacts/control_force_v62/teacher_failure_v83'
SOURCE = OUT.parent / 'teacher_pilot_v82'


def first(records, condition):
    return next((float(r['time']) for r in records if condition(r)), None)


def main():
    target = OUT / 'failure_evidence.json'
    assert not target.exists(), 'Preserve previous audits.'
    summary = json.loads((SOURCE / 'summary.json').read_text(encoding='utf-8-sig'))
    rows = []
    for result in summary['rows']:
        if result['admitted']:
            continue
        file = SOURCE / (result['task'] + '_' + result['mode'].replace('-', '_') + '.mat')
        data = loadmat(file, variable_names=['records', 'plans'], simplify_cells=True)
        records = np.atleast_1d(data['records'])
        plans = np.atleast_1d(data['plans'])
        collision_h = lambda r: np.asarray(r['diagnostic']['cbf']['stateMargins']).reshape(-1)[-1]
        collision_psi = lambda r: np.asarray(r['diagnostic']['cbf']['psi1']).reshape(-1)[-1]
        fail = next(r for r in records if r['diagnostic']['usedFallback'])
        check = fail['diagnostic']['interval_projection']['interval']
        rejected_checks = [p for p in plans if isinstance(p.get('previous_reference_check'), dict)
                           and not p['previous_reference_check']['accepted']]
        rows.append(dict(task=result['task'], mode=result['mode'], subtype=result['subtype'],
                         source_sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                         first_negative_collision_psi1=first(records, lambda r: collision_psi(r) < 0),
                         first_negative_collision_h=first(records, lambda r: collision_h(r) < 0),
                         first_qp_failure=float(fail['time']),
                         first_logged_reference_rejection=float(rejected_checks[0]['time']) if rejected_checks else None,
                         original_qp_feasible=bool(fail['diagnostic']['original_qp_feasible']),
                         hold_mechanical_violation=float(check['internal_check']['mechanical_violation']),
                         hold_collision_violation=float(check['internal_check']['collision_violation']),
                         metrics=result['metrics'], simulation_error=result['simulation_error']))
    report = dict(completed=True, formal=False, rows=rows,
                  conclusions=[
                      'All six orbit episodes lose the collision psi1 condition at 6.18 s and first fail the hold/QP gate at 6.26 s.',
                      'Goal hold revalidation originally covers only 0.2 s after arrival; the observation-only 1.2 s replay rejects at 5.48 s.',
                      'The crossing nonideal episode fails a mechanical hold constraint at 13.78 s while collision conditions remain valid.'
                  ],
                  distinctions='Short-horizon warning is demonstrated by replay; closed-loop repair success requires new Simscape runs.',
                  references=['https://arxiv.org/abs/2403.19122', 'https://arxiv.org/abs/2304.01639'])
    target.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print('Preserved failure audit:', len(rows), 'episodes')


if __name__ == '__main__':
    main()
