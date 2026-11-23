"""Reuse exact same-task completed results; changed durations get new task IDs."""
import json
from pathlib import Path
import shutil
import time

root=Path(__file__).resolve().parents[2]
old=root/'results/chapter3/payload_condition_expansion_v1_20260926'
out=root/'results/chapter3/payload_condition_expansion_v2_longtime_20260926'
manifest=json.loads((root/'analysis/payload_expansion/candidates.json').read_text(encoding='utf-8-sig'))
for name in ['accepted','checks','new_attempts']:(out/name).mkdir(parents=True,exist_ok=True)
for path in (old/'checks').glob('attempt_*.json'):shutil.copy2(path,out/'checks'/path.name)
for path in (old/'accepted').glob('formal_*.mat'):shutil.copy2(path,out/'accepted'/path.name)
reused=[]
previous=json.loads((old/'selected_tasks.json').read_text(encoding='utf-8-sig'))
previous={t['taskId']:t for t in previous['tasks']}
for task in manifest['tasks']:
    task_id=task['taskId'];check=old/'checks'/f'{task_id}.json'
    if task_id not in previous:continue
    comparable={k:v for k,v in task.items() if k not in ['baseConditionId','specHash']}
    old_comparable={k:v for k,v in previous[task_id].items() if k!='specHash'}
    assert comparable==old_comparable, 'Task ID reuse requires exact unchanged physical specification'
    if check.exists():
        shutil.copy2(check,out/'checks'/check.name)
        accepted=old/'accepted'/f'{task_id}.mat'
        if accepted.exists():shutil.copy2(accepted,out/'accepted'/accepted.name)
        reused.append(task_id)
    else:
        attempt=old/'new_attempts'/f'{task_id}.mat'
        if attempt.exists():shutil.copy2(attempt,out/'new_attempts'/attempt.name)
selection=dict(tasks=manifest['tasks'],count=300,selectionRule=manifest['selection'],
    endpointAuditSource=str(old/'selected_tasks.json'),endpointAudit='unchanged endpoints and payloads; rechecked again before each solve')
(out/'selected_tasks.json').write_text(json.dumps(selection,ensure_ascii=False),encoding='utf-8')
(out/'duration_revision.json').write_text(json.dumps(dict(previousRun=str(old),updatedUnix=time.time(),
    durations=manifest['durations'],reusedExactCompletedTasks=reused,newTaskIdForChangedDuration=True),indent=2),encoding='utf-8')
(old/'worker_status.json').write_text(json.dumps(dict(status='superseded_by_user_duration_revision',updatedUnix=time.time(),nextRun=str(out))),encoding='utf-8')
(old/'finish_status.json').write_text(json.dumps(dict(status='superseded_by_user_duration_revision',nextRun=str(out))),encoding='utf-8')
print(json.dumps(dict(reusedCompleted=len(reused),targetConditions=300,durations=manifest['durations'])))
