import sys,json,shutil,datetime,csv
from pathlib import Path
from collections import Counter
from prepare_cleanup import *
def main():
 manifest=read(OUT/'deletion_manifest.json');results=[json.loads(line) for line in (OUT/'deletion_results.jsonl').read_text(encoding='utf8').splitlines()]
 assert len(results)==len(manifest['files'])
 deleted={x['path']:x for x in results if x['status']=='deleted'};skipped=[x for x in results if x['status']!='deleted'];cache={}
 def checked(item):
  p=Path(item['path']);assert p.exists() and p.stat().st_size==item['bytes'],str(p)
  if str(p) not in cache:cache[str(p)]=digest(p)
  assert cache[str(p)]==item['sha256'],str(p)
 for path,item in deleted.items():assert not Path(path).exists() or item['category']=='cache',path
 protected=read(OUT/'protected_before.json')
 for item in protected['files']:checked(item)
 archive=protected['archive'];assert digest(Path(archive['archive']))==archive['sha256']
 for item in read(OUT/'canonical_replacements.json'):checked(item)
 for item in read(OUT/'qp_retained.json'):checked(item)
 # Independently ensure each logged anomaly and held frame still has all
 # previously captured matrices, not merely that a selection list exists.
 anomalies=holds=0
 for index in (EVIDENCE/'experiments').glob('*/qp_snapshot_index.jsonl'):
  folder=index.parent;index_rows=[json.loads(s) for s in index.read_text(encoding='utf8').splitlines() if s.strip()]
  available={(int(r['controlStep']),int(r['localIteration'])):r for r in index_rows}
  attempts=folder/'solver_attempts.csv';status=folder/'solver_status.csv'
  if attempts.exists() and status.exists():
   tol=read(folder/'configuration.json')['qpAcceptanceTolerance']
   for a in rows(attempts):
    bad=not finite(a['exitflag']) or float(a['exitflag'])<=0 or not truth(a['candidateValidity']) or not finite(a['scaledResidual']) or float(a['scaledResidual'])>tol or not all(finite(a.get(f'candidate{i}')) for i in range(1,7))
    if bad:
     anomalies+=1;key=(int(a['controlStep']),int(a['localIteration']));assert key in available,(folder.name,key)
     assert (folder/available[key]['file']).exists(),(folder.name,key)
   held={int(f['controlStep']) for f in rows(status) if truth(f['heldPreviousCommand'])};holds+=len(held)
   for r in index_rows:
    if r['controlStep'] in held:assert (folder/r['file']).exists(),(folder.name,r)
  for r in index_rows:
   if finite(r.get('nonlinearFutureMinimum')) and r['nonlinearFutureMinimum']<0:assert (folder/r['file']).exists()
  current=[]
  retained_reason={x['originalIndex']:x for x in read(OUT/'retained_indexes'/f'{folder.name}.json')}
  for n,r in enumerate(index_rows,1):
   if (folder/r['file']).exists():current.append(dict(**r,originalIndex=n,retentionReasons=retained_reason.get(n,{}).get('retentionReasons',['retained after deletion skip'])))
  write(OUT/'retained_indexes'/f'{folder.name}.json',current)
 # Cache checks are read-only; they do not rewrite frozen result files.
 sys.path.insert(0,str(EVIDENCE/'analysis'));sys.path.insert(0,str(V8/'lib/python_deps'))
 from statistics_core import cache_valid,cache_signature
 checked_cases=[]
 for category in ['corrected_baseline','experiments']:
  for summary in (EVIDENCE/category).glob('*/summary.json'):
   assert cache_valid(summary.parent,cache_signature(summary.parent)),str(summary.parent)
   checked_cases.append(str(summary.parent.relative_to(EVIDENCE)))
 old=read(OUT/'inventory_before.json');unexpected=[]
 for item in old['files']:
  if not Path(item['path']).exists() and item['path'] not in deleted:unexpected.append(item['path'])
 assert not unexpected,unexpected
 counts=Counter();sizes=Counter()
 for item in deleted.values():counts[item['category']]+=1;sizes[item['category']]+=item['bytes']
 retained_archives=read(OUT/'archive_decisions.json')
 newly_preserved=sum(x.get('uniqueUncompressedBytes',0) for x in retained_archives if x['action']=='delete')
 original_freed=sum(x['bytes'] for x in deleted.values() if x['category']!='verification_temporary')
 report=dict(passed=True,finishedAt=datetime.datetime.now().astimezone().isoformat(),deletedFiles=len(deleted),skippedFiles=len(skipped),
   categoryCounts=dict(counts),categoryBytes=dict(sizes),originalContentDeletedBytes=original_freed,newlyPreservedArchiveBytes=newly_preserved,
   netContentReductionBeforeAuditOverhead=original_freed-newly_preserved,
   diskFreeAtInventory=old['diskFreeBytes'],diskFreeBeforeDeletion=manifest['diskFreeBeforeDeletion'],diskFreeAfter=shutil.disk_usage(ROOT).free,
   protectedFileHashesPassed=len(protected['files']),canonicalReplacementHashesPassed=len(read(OUT/'canonical_replacements.json')),
   retainedQpHashesPassed=len(read(OUT/'qp_retained.json')),abnormalAttemptsCovered=anomalies,heldFramesCovered=holds,
   validStatisticsCaches=len(checked_cases),latestArchiveSHA256=archive['sha256'],unexpectedMissingFiles=unexpected,
   retainedArchives=[x for x in retained_archives if x['action']=='retain'],skipped=skipped)
 write(OUT/'verification.json',report)
 with (OUT/'deleted_files.csv').open('w',encoding='utf-8-sig',newline='') as f:
  writer=csv.DictWriter(f,fieldnames=['path','category','bytes','sha256','reason','replacement']);writer.writeheader()
  lookup={x['path']:x for x in manifest['files']}
  for path,item in deleted.items():writer.writerow({k:lookup[path].get(k,'') for k in writer.fieldnames})
 print('POST_CLEANUP_VERIFIED',len(deleted),'deleted;',len(skipped),'skipped;',anomalies,'anomalies retained;',len(checked_cases),'caches valid',flush=True)
if __name__=='__main__':main()
