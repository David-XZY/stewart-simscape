"""Create new training-only conditions without consulting solver outcomes."""
import json
import collections
import copy
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'chapter3/payload_generalization'))
from pg_formal_tasks import formal_tasks
from pg_core import digest, write

def main():
    previous=ROOT/'analysis/payload_expansion/selected_tasks_snapshot.json'
    if previous.exists():
        previous_tasks=json.loads(previous.read_text(encoding='utf-8-sig'))['tasks']
        groups=collections.defaultdict(list)
        for t in previous_tasks:groups[(t['family'],t['taskClass'],t['shape'])].append(t)
        bases=[t for group in groups.values() for t in group[:5]]
        bases.sort(key=lambda t:digest(t['taskId']))
        tasks=[]
        for base in bases:
            for duration in [15,12.5,10,7.5,5]:
                t=copy.deepcopy(base)
                t['baseConditionId']=base['taskId']
                t['parameters'][3]=duration
                if duration!=base['parameters'][3]:t['taskId']+=f'_T{int(duration*10):03d}'
                t.pop('specHash',None);t['specHash']=digest(t)
                tasks.append(t)
        assert len(tasks)==300 and len({t['baseConditionId'] for t in tasks})==60
        assert collections.Counter(t['parameters'][3] for t in tasks)=={5:60,7.5:60,10:60,12.5:60,15:60}
        write(ROOT/'analysis/payload_expansion/candidates.json',dict(tasks=tasks,seed=20260926,targetPerStratum=25,
            strata='family x taskClass x payload shape',targetConditions=300,durations=[5,7.5,10,12.5,15],
            selection='First five endpoint-valid base conditions per stratum in frozen hash order, independent of solver outcome; each crosses all five durations',taskHash=digest(tasks)))
        print('Prepared 300 conditions: 60 bases x 5 durations; 60 each at 5, 7.5, 10, 12.5 and 15 seconds.')
        return
    tasks = []
    for t in formal_tasks(600, seed=20260926)['tasks']:
        if t['split'] != 'train':
            continue
        t['taskId'] = t['taskId'].replace('formal_train_', 'expand_20260926_train_')
        t['baseSceneId'] = t['baseSceneId'].replace('formal_train_', 'expand_20260926_train_')
        t.pop('specHash', None)
        t['specHash'] = digest(t)
        tasks.append(t)
    tasks.sort(key=lambda t: digest(t['taskId']))
    assert len(tasks) == len({t['taskId'] for t in tasks}) == 600
    assert all(int(__import__('hashlib').sha256(t['payloadRegionId'].encode()).hexdigest()[:8],16)%10<7 for t in tasks)
    write(ROOT/'analysis/payload_expansion/candidates.json', {
        'tasks': tasks, 'seed': 20260926, 'targetPerStratum': 10,
        'strata': 'family x taskClass x payload shape', 'targetConditions': 120,
        'selection': 'deterministic hash order; endpoint hard-constraint screen; no solve-outcome screen',
        'taskHash': digest(tasks)})
    print('Prepared 600 train-only candidate conditions; select 10 endpoint-valid per 12 strata.')

if __name__ == '__main__':
    main()
