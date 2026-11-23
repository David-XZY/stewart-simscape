"""Apply an explicit deduplication policy while preserving validation labels."""
import argparse
import csv
import hashlib
from pathlib import Path, PureWindowsPath
import numpy as np
from analyze_admission import read, write, select

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('audit',type=Path);ap.add_argument('parent',type=Path)
    ap.add_argument('--threshold',type=float,required=True)
    ap.add_argument('--cap',type=int,default=4)
    ap.add_argument('--all-qualified',action='store_true')
    args=ap.parse_args()
    census=read(args.audit/'candidates.json'); candidates=census['candidates']
    parent=read(args.parent/'dataset.json');library=read(args.parent/'library.json')
    by_task={}
    for row in candidates:by_task.setdefault(row['taskId'],[]).append(row)
    with (args.audit/'pairs.csv').open(encoding='utf-8-sig',newline='') as f:
        pairs=list(csv.DictReader(f))
    distances={frozenset((p['labelA'],p['labelB'])):float(p['rawModeRms']) for p in pairs}
    old={r['taskId']:PureWindowsPath(r['source']).stem for r in library['records'] if r['fittedPassed']}
    selected=[]
    for task,g in by_task.items():
        if g[0]['split']=='validation':
            selected.append(next(x for x in g if x['labelId']==old[task]))
        elif args.all_qualified:selected.extend(sorted(g,key=lambda x:x['labelId']))
        else:selected.extend(select(g,distances,args.threshold,args.cap))
    selected_ids={r['labelId'] for r in selected}
    assert set(old.values())<=selected_ids,'Original accepted representatives must be retained.'
    assert all(r['physicalRecheckPassed'] for r in selected)
    assert len({r['taskId'] for r in selected if r['split']=='train'})==175
    assert sum(r['split']=='validation' for r in selected)==53
    parent_hash=hashlib.sha256((args.parent/'dataset.json').read_bytes()).hexdigest()
    assert parent_hash==census['identity']['datasetSha256']
    policy=dict(version='payload_admission_v2',selection='all_physically_qualified' if args.all_qualified else 'within_task_raw_geometry_dedup',
                originalThreshold=.15,threshold=None if args.all_qualified else args.threshold,
                maximumLabelsPerTask=None if args.all_qualified else args.cap,ordering='original_source_order',
                validationPolicy='preserve_original_53_labels_and_all_120_evaluation_conditions',
                physicalGate='unchanged',fitModeErrorMaximum=.15,originalRepresentativesPreserved=True,
                taskEqualTraining='existing pg_learning samples each training condition equally')
    output=dict(parentDatasetSha256=parent_hash,candidatesSha256=hashlib.sha256((args.audit/'candidates.json').read_bytes()).hexdigest(),
                policy=policy,selectedLabels=[r['labelId'] for r in selected],
                counts=dict(train=sum(r['split']=='train' for r in selected),validation=53,
                            trainingConditions=175,validationConditions=53),
                addedTrainingLabels=[r['labelId'] for r in selected if r['split']=='train' and r['labelId']!=old[r['taskId']]])
    write(args.audit/'selection.json',output)
    print(output['counts']);print('added',len(output['addedTrainingLabels']))

if __name__=='__main__':main()
