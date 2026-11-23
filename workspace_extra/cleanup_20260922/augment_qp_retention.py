import sys,os
from pathlib import Path
from collections import defaultdict
from prepare_cleanup import *
sys.path.insert(0,str(V8/'lib/python_deps'))
from scipy.io import loadmat

def main():
 candidates={x['path']:x for x in read(OUT/'qp_candidates.json')};kept={x['path']:x for x in read(OUT/'qp_retained.json')};extra=defaultdict(set);notes=[]
 def keep(path,reason):
  p=str(Path(path).resolve())
  if p in candidates or p in kept:extra[p].add(reason)
 for file in (EVIDENCE/'replay_inputs').glob('*.mat'):
  data=loadmat(file,variable_names=['snapshots'],simplify_cells=True);snapshots=data.get('snapshots',[])
  if isinstance(snapshots,dict):snapshots=[snapshots]
  for snap in snapshots:
   if isinstance(snap,dict) and isinstance(snap.get('sourceChunk'),str):keep(snap['sourceChunk'],'explicit reference in '+file.name)
 for file in EVIDENCE.glob('replay*/classification.json'):
  proofs=read(file).get('proofs',[])
  if isinstance(proofs,dict):proofs=[proofs]
  for proof in proofs:
   path=proof.get('sourceChunk')
   if isinstance(path,str):keep(path,'explicit reference in '+file.parent.name)
 for index in (EVIDENCE/'experiments').glob('*/qp_snapshot_index.jsonl'):
  folder=index.parent;records=[__import__('json').loads(s) for s in index.read_text(encoding='utf8').splitlines() if s.strip()]
  for r in records:
   for field in ['scaledApproximationError','hessianReciprocalCondition','hessianSymmetryError','nonlinearFutureMinimum','linearFutureMinimum']:
    if field in r and not finite(r[field]):keep(folder/r['file'],'missing/nonfinite snapshot diagnostic: '+field)
  for category in [-2,0,1]:
   matches=[r for r in records if r['exitflag']>0] if category==1 else [r for r in records if r['exitflag']==category]
   if matches:keep(folder/matches[-1]['file'],'unchanged replay selector: last occurrence of flag category')
  zero_initial=[r for r in records if r['exitflag']==0 and r['localIteration']==0]
  if zero_initial:keep(folder/zero_initial[-1]['file'],'unchanged replay selector: last initial flag zero')
  physical=folder/'constraint_residuals.csv';status=folder/'solver_status.csv'
  if not physical.exists() or not status.exists():continue
  values=rows(physical);frames=rows(status);record_steps={r['controlStep'] for r in records}
  for group,prefix in [('length','length_'),('speed','speed_'),('acceleration','acceleration_'),('singularity','sigma_'),('collision','collision_'),('force','actual_force_'),('rate','actual_rate_')]:
   fields=[key for key in values[0] if key.startswith(prefix)]
   minima=[(float(row[key]),float(row['time']),key) for row in values for key in fields if finite(row[key])]
   if not minima:continue
   residual,time,key=min(minima)
   frame=max((f for f in frames if float(f['time'])<=time+1e-10),key=lambda f:float(f['time']),default=frames[0]);step=int(frame['controlStep'])
   selected=[r for r in records if r['controlStep']==step]
   if not selected:
    distances=sorted(record_steps,key=lambda s:(abs(s-step),s))[:2]
    selected=[r for r in records if r['controlStep'] in distances]
   for r in selected:keep(folder/r['file'],'actual constraint-group extremum: '+group)
   notes.append(dict(case=folder.name,group=group,field=key,residual=residual,time=time,controlStep=step,exactFrameSnapshotExisted=step in record_steps,retainedSteps=sorted({r['controlStep'] for r in selected})))
 for path,reasons in extra.items():
  if path in candidates:
   item=candidates.pop(path);item['reasons']=[];item.pop('category',None);item.pop('reason',None);item.pop('replacement',None);kept[path]=item
  kept[path]['reasons']=sorted(set(kept[path]['reasons'])|reasons)
 write(OUT/'qp_candidates.json',list(candidates.values()));write(OUT/'qp_retained.json',list(kept.values()));write(OUT/'physical_extrema_retention.json',notes)
 for index in (EVIDENCE/'experiments').glob('*/qp_snapshot_index.jsonl'):
  records=[__import__('json').loads(s) for s in index.read_text(encoding='utf8').splitlines() if s.strip()];current=[]
  for n,r in enumerate(records,1):
   path=str((index.parent/r['file']).resolve())
   if path in kept:current.append(dict(**r,originalIndex=n,retentionReasons=kept[path]['reasons']))
  write(OUT/'retained_indexes'/f'{index.parent.name}.json',current)
 print('FINAL_QP_SELECTION',len(candidates),sum(x['bytes'] for x in candidates.values())/2**30,'GiB; retained',len(kept),flush=True)
if __name__=='__main__':main()
