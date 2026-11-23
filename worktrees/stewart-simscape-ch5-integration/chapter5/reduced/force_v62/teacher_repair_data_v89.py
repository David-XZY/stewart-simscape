"""Materialize only complete admitted repair runs, keeping every failed identity."""
from pathlib import Path
import hashlib
import json
import time
import numpy as np
from scipy.io import loadmat, savemat

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'chapter5/artifacts/control_force_v62/teacher_repair_v89'
SOURCE = OUT.parent / 'teacher_pilot_v82'


def main():
    assert not (OUT / 'dataset_report.json').exists(), 'Preserve materialization attempts.'
    deadline = time.monotonic() + 10800
    while True:
        try:
            summary = json.loads((OUT / 'summary.json').read_text(encoding='utf-8-sig'))
        except (OSError, json.JSONDecodeError):
            summary = {}
        if summary.get('completed'):
            break
        if time.monotonic() > deadline:
            raise TimeoutError('All three explicit result states')
        time.sleep(5)
    identity = json.loads((OUT / 'identity.json').read_text(encoding='utf-8-sig'))
    for name, expected in {**identity['original_sources'], **identity['sources']}.items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == expected, name
    tasks = {t['id']: t for t in json.loads((SOURCE / 'tasks.json').read_text(encoding='utf-8-sig'))}
    contexts, delta, force, task_ids, modes, times, manifest = [], [], [], [], [], [], []
    for row in summary['rows']:
        task = tasks[row['task']]
        path = OUT / (row['task'] + '_' + row['mode'].replace('-', '_') + '.mat')
        entry = dict(task=row['task'], split=task['split'], subtype=task['subtype'], mode=row['mode'],
                     admitted=row['admitted'], status=row['status'], windows=0,
                     source_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        manifest.append(entry)
        if not row['admitted']:
            continue
        records = np.atleast_1d(loadmat(path, variable_names=['records'], simplify_cells=True)['records'])
        clock = np.array([r['time'] for r in records])
        U = np.stack([np.asarray(r['u']).reshape(6) for r in records])
        previous = np.stack([np.asarray(r['previous_u']).reshape(6) for r in records])
        assert len(records) == 751 and abs(clock[-1] - 15) < 1e-8
        assert np.max(np.abs(np.diff(clock) - .02)) < 1e-9
        assert np.max(np.abs(previous[1:] - U[:-1])) < 1e-10
        assert np.max(np.abs(np.asarray(records[0]['q']) - task['q0'])) < 1e-7
        boundaries = np.r_[np.asarray(task['goal_times']).reshape(-1)[1:], np.asarray(task['visible_times']).reshape(-1)]
        for index in range(0, len(records) - 60 + 1, 5):
            if np.any((boundaries > clock[index] + 1e-10) & (boundaries <= clock[index] + 1.2 + 1e-10)):
                continue
            r = records[index]
            x = np.asarray(r['x']).reshape(-1)
            state = np.zeros(30)
            state[:len(x)] = x
            condition = np.r_[state, previous[index], np.asarray(r['goal']).reshape(6),
                              np.asarray(r['observed']).reshape(176, order='F'), float(row['mode'] == 'nonideal-force')]
            commands = U[index:index+60]
            changes = np.diff(np.r_[previous[index][None, :], commands], axis=0)
            assert condition.shape == (219,) and np.isfinite(condition).all() and np.isfinite(commands).all()
            contexts.append(condition);force.append(commands.reshape(360));delta.append(changes.reshape(360))
            task_ids.append(task['id']);modes.append(row['mode']);times.append(clock[index]);entry['windows'] += 1
    report = dict(completed=True, formal=False, rows=manifest, result_states=len(manifest),
                  admitted_runs=sum(bool(r['admitted']) for r in manifest), windows=len(contexts),
                  normalization_applied=False, model_trained=False,
                  label_rule='Original final safety-QP commands/increments, no padding or repair of labels.',
                  partition_rule='Complete source task identity; these repair tasks belong only to training.',
                  batch_is_additional_training_data=True)
    if contexts:
        savemat(OUT / 'dataset_raw.mat', dict(context=np.asarray(contexts).T, delta=np.asarray(delta).T,
                force=np.asarray(force).T, task=np.asarray(task_ids, dtype=object)[None, :],
                split=np.full((1, len(task_ids)), 'train', dtype=object), mode=np.asarray(modes, dtype=object)[None, :],
                time=np.asarray(times)[None, :]), do_compression=True)
        report['dataset_sha256'] = hashlib.sha256((OUT / 'dataset_raw.mat').read_bytes()).hexdigest()
    (OUT / 'dataset_report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print('Repair training windows:', len(contexts), 'admitted runs:', report['admitted_runs'], flush=True)


if __name__ == '__main__':
    main()
