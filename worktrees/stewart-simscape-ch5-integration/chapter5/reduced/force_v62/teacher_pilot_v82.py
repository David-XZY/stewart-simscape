"""Materialize complete successful teacher episodes; split by whole task.

Labels are original final-QP force increments, without tail repair or padding.
Retain every failed run in the manifest and avoid unseen appearance events.
"""
from pathlib import Path
import hashlib, json, time
import numpy as np
import scipy.io as sio

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'chapter5/artifacts/control_force_v62/teacher_pilot_v82'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False),encoding='utf-8')

def main():
    assert not (OUT/'dataset_report.json').exists(),'Preserve materialization attempts.'
    start=notice=time.monotonic()
    while True:
        path=OUT/'summary.json';summary=json.loads(path.read_text(encoding='utf-8-sig')) if path.exists() else {}
        if summary.get('completed'):break
        if time.monotonic()-start>7200:raise TimeoutError('18 moving-teacher result states')
        if time.monotonic()-notice>30:print('Moving teacher results',len(summary.get('rows',[])),'/ 18',flush=True);notice=time.monotonic()
        time.sleep(2)
    identity=json.loads((OUT/'identity.json').read_text(encoding='utf-8-sig'))
    for name,expected in identity['sources'].items():
        assert sha(ROOT/name)==expected,f'Frozen source changed: {name}'
    tasks={t['id']:t for t in json.loads((OUT/'tasks.json').read_text(encoding='utf-8-sig'))}
    contexts=[];increments=[];forces=[];source_ids=[];splits=[];modes=[];times=[];manifest=[]
    horizon=60;stride=5
    for result in summary['rows']:
        task=tasks[result['task']];source=OUT/f"{result['task']}_{result['mode'].replace('-','_')}.mat"
        entry=dict(task=result['task'],split=result['split'],subtype=result['subtype'],mode=result['mode'],
            admitted=bool(result['admitted']),status=result['status'],source_sha256=sha(source),windows=0,event_crossings_excluded=0)
        manifest.append(entry)
        if not entry['admitted']:continue
        records=np.atleast_1d(sio.loadmat(source,variable_names=['records'],simplify_cells=True)['records'])
        command=np.stack([np.asarray(r['u']).reshape(6) for r in records]);previous=np.stack([np.asarray(r['previous_u']).reshape(6) for r in records])
        clock=np.array([r['time'] for r in records],dtype=float)
        initial_error=np.max(np.abs(np.asarray(records[0]['q'])-np.asarray(task['q0'])))
        assert len(clock)>=751 and abs(clock[-1]-15)<1e-8 and np.max(np.abs(np.diff(clock)-.02))<1e-9
        assert np.max(np.abs(previous[1:]-command[:-1]))<1e-10 and initial_error<1e-7
        boundaries=np.r_[np.asarray(task['goal_times']).reshape(-1)[1:],np.asarray(task['visible_times']).reshape(-1)]
        boundaries=boundaries[(boundaries>0)&(boundaries<=15)]
        for index in range(0,len(records)-horizon+1,stride):
            begin=clock[index];end=begin+horizon*.02
            if np.any((boundaries>begin+1e-10)&(boundaries<=end+1e-10)):
                entry['event_crossings_excluded']+=1;continue
            row=records[index];x=np.asarray(row['x']).reshape(-1);state=np.zeros(30);state[:len(x)]=x
            context=np.r_[state,previous[index],np.asarray(row['goal']).reshape(6),
                np.asarray(row['observed']).reshape(176,order='F'),float(result['mode']=='nonideal-force')]
            U=command[index:index+horizon];D=np.diff(np.r_[previous[index][None,:],U],axis=0)
            assert len(context)==219 and U.shape==(60,6) and np.isfinite(context).all() and np.isfinite(U).all()
            contexts.append(context);forces.append(U.reshape(360));increments.append(D.reshape(360));
            source_ids.append(task['id']);splits.append(task['split']);modes.append(result['mode']);times.append(begin);entry['windows']+=1
    report=dict(completed=True,formal=False,expected_runs=18,result_states=len(manifest),admitted_runs=sum(r['admitted'] for r in manifest),
        unique_admitted_tasks=len({r['task'] for r in manifest if r['admitted']}),rows=manifest,
        label_rule='original final-QP commands/increments; no repair, padding or sample crossing an unannounced appearance',
        normalized_from_training_only=True,moving_model_trained=False,implementation_sha256=sha(__file__))
    if not contexts:
        report.update(materialized=False,error='No admitted complete teacher runs');write(OUT/'dataset_report.json',report);raise RuntimeError(report['error'])
    C=np.asarray(contexts);D=np.asarray(increments);U=np.asarray(forces);partition=np.asarray(splits);train=partition=='train'
    assert train.any() and (partition=='validation').any()
    assert not set(np.asarray(source_ids)[train])&set(np.asarray(source_ids)[~train])
    context_mean=C[train].mean(0);context_std=np.maximum(C[train].std(0),1e-3);delta_mean=D[train].mean(0);delta_std=np.maximum(D[train].std(0),1e-3)
    supported=C[train].std(0)>=1e-8
    sio.savemat(OUT/'dataset.mat',dict(context=C.T,delta=D.T,force=U.T,task=np.asarray(source_ids,dtype=object)[None,:],
        split=partition.astype(object)[None,:],mode=np.asarray(modes,dtype=object)[None,:],time=np.asarray(times)[None,:],
        context_mean=context_mean[:,None],context_std=context_std[:,None],delta_mean=delta_mean[:,None],delta_std=delta_std[:,None],
        context_supported=supported[:,None]),do_compression=True)
    velocity_columns=np.array([42+22*j+k for j in range(8) for k in [15,16,17]])
    report.update(materialized=True,total_windows=len(C),training_windows=int(train.sum()),validation_windows=int((~train).sum()),
        context_supported_count=int(supported.sum()),supported_velocity_columns=(velocity_columns[supported[velocity_columns]]+1).tolist(),
        dataset_sha256=sha(OUT/'dataset.mat'))
    write(OUT/'dataset_report.json',report);print('Moving teacher dataset',report['training_windows'],report['validation_windows'],'windows',flush=True)

if __name__=='__main__':main()
