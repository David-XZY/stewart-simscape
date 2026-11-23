from prepare_cleanup import *
from collections import Counter

counts=Counter()
preexisting_missing=[]
for index in (EVIDENCE/'experiments').glob('*/qp_snapshot_index.jsonl'):
 folder=index.parent
 records=[json.loads(line) for line in index.read_text(encoding='utf8').splitlines() if line.strip()]
 assert index.read_bytes()==(OUT/'original_qp_indexes'/folder.name/index.name).read_bytes()
 def check(record,kind):
  assert (folder/record['file']).is_file(),(folder.name,kind,record)
  counts[kind]+=1
 for record in sorted([r for r in records if finite(r.get('scaledApproximationError'))],key=lambda r:(-r['scaledApproximationError'],r['controlStep'],r['localIteration']))[:3]:
  check(record,'top_three_error_attempts')
 if (folder/'solver_status.csv').exists():
  frames=rows(folder/'solver_status.csv')
  steps={int(frames[0]['controlStep'])}|{int(f['controlStep']) for f in frames[-3:]}
  for record in records:
   if record['controlStep'] in steps:check(record,'endpoint_attempts')
 if (folder/'solver_attempts.csv').exists():
  attempts=rows(folder/'solver_attempts.csv')
  indexed={(r['controlStep'],r['localIteration']):r for r in records}
  for field in [key for key in attempts[0] if key.startswith('nonlinearMinimum_')]:
   values=[a for a in attempts if finite(a.get(field))]
   if values:
    worst=min(values,key=lambda a:float(a[field]))
    key=(int(worst['controlStep']),int(worst['localIteration']))
    if key in indexed:
     check(indexed[key],'nonlinear_group_worst_attempts')
    else:
     same_frame=[r for r in records if r['controlStep']==key[0]]
     preexisting_missing.append(dict(case=folder.name,field=field,controlStep=key[0],localIteration=key[1],existingFrameSnapshots=len(same_frame),otherAttemptsInFrameRetained=all((folder/r['file']).exists() for r in same_frame)))
 for note in read(OUT/'physical_extrema_retention.json'):
  if note['case']==folder.name:
   selected=[r for r in records if r['controlStep'] in note['retainedSteps']]
   assert selected
   for record in selected:check(record,'physical_group_extremum_attempts')
 counts['cases']+=1
gaps=[x for x in preexisting_missing if not x['otherAttemptsInFrameRetained']]
write(OUT/'representative_coverage.json',dict(passed=not gaps,capturedRepresentativeChecksPassed=True,counts=dict(counts),originalIndexesUnchanged=True,preexistingUnindexedWorstAttempts=preexisting_missing,wholeFrameRetentionGaps=gaps))
print('REPRESENTATIVE_COVERAGE',dict(counts),'whole-frame retention gaps:',len(gaps))
