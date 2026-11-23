import os,sys,json,csv,hashlib,stat,math,zipfile,shutil
from pathlib import Path
from collections import defaultdict
ROOT=Path(r'D:\NPU\Matlab_code');OUT=ROOT/'cleanup_20260922'
V8=ROOT/'stewart-simscape-tracking-mpc-v8-sqp-statistics';EVIDENCE=V8/'results/sqp_statistics_20260922'
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def write(p,value):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
def digest(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
 return h.hexdigest()
def regular(p):return not bool(p.lstat().st_file_attributes & stat.FILE_ATTRIBUTE_REPARSE_POINT)
def rows(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def finite(x):
 try:return math.isfinite(float(x))
 except (TypeError,ValueError):return False
def truth(x):return str(x).lower() in ('1','true')
def main():
 verification=read(EVIDENCE/'package_validation.json');archive=Path(verification['archive'])
 assert archive.exists() and digest(archive)==verification['sha256']
 with zipfile.ZipFile(archive) as z:assert z.testzip() is None
 write(OUT/'latest_archive_verified.json',dict(passed=True,path=str(archive),sha256=verification['sha256'],crcPassed=True,bytes=archive.stat().st_size))
 print('LATEST_ARCHIVE_VERIFIED',flush=True)
 selection=read(EVIDENCE/'selection.json')
 for item in selection['sourceFiles']:assert digest(V8/item['path'])==item['sha256'],item['path']
 manifest=read(EVIDENCE/'delivery_manifest.json')
 protected={str(V8/x['path']):x for x in manifest['includedFiles']}
 for path,item in protected.items():assert digest(Path(path))==item['sha256'],path
 write(OUT/'protected_before.json',dict(archive=verification,files=[dict(path=p,bytes=x['bytes'],sha256=x['sha256']) for p,x in protected.items()],frozenSelectionSHA256=digest(EVIDENCE/'selection.json')))
 print('FROZEN_AND_PACKAGED_FILES_VERIFIED',len(protected),flush=True)
 inventory=[];skipped=[];total=0
 for base,dirs,files in os.walk(ROOT,followlinks=False):
  parent=Path(base);allowed=[]
  for name in dirs:
   p=parent/name
   if p==OUT or name=='.git' or not regular(p):skipped.append(str(p))
   else:allowed.append(name)
  dirs[:]=allowed
  for name in files:
   p=parent/name
   if not regular(p):skipped.append(str(p));continue
   s=p.stat();total+=s.st_size
   inventory.append(dict(path=str(p),bytes=s.st_size,mtimeNs=s.st_mtime_ns))
 write(OUT/'inventory_before.json',dict(root=str(ROOT),managedBytes=total,diskFreeBytes=shutil.disk_usage(ROOT).free,files=inventory,excluded=skipped))
 print('INVENTORY',len(inventory),total,flush=True)
 known={str(V8/x['path']):x for x in manifest['omittedPerFrameQpChunks']}
 candidates=[];kept=[];case_reports=[]
 for index in sorted((EVIDENCE/'experiments').glob('*/qp_snapshot_index.jsonl')):
  folder=index.parent;records=[json.loads(s) for s in index.read_text(encoding='utf8').splitlines() if s.strip()]
  paths={r['file'].replace('\\','/') for r in records};reasons=defaultdict(set)
  def retain(r,reason):reasons[r['file'].replace('\\','/')].add(reason)
  log=folder/'solver_attempts.csv';status=folder/'solver_status.csv'
  complete=(folder/'summary.json').exists() and log.exists() and status.exists()
  if not complete:
   for r in records:retain(r,'incomplete trial: full candidate/hold logs unavailable; preserve evidence')
  else:
   config=read(folder/'configuration.json');tol=config['qpAcceptanceTolerance'];attempts=rows(log);frames=rows(status)
   by_id={(int(r['controlStep']),int(r['localIteration'])):r for r in records}
   by_frame=defaultdict(list)
   for r in records:by_frame[int(r['controlStep'])].append(r)
   abnormal=set();no_snapshot=[]
   for a in attempts:
    key=(int(a['controlStep']),int(a['localIteration']))
    bad=(not finite(a['exitflag']) or float(a['exitflag'])<=0 or not truth(a['candidateValidity']) or not finite(a['scaledResidual']) or float(a['scaledResidual'])>tol)
    numeric=[a.get(f'candidate{i}','') for i in range(1,7)]
    bad=bad or not all(finite(x) for x in numeric)
    if bad:
     abnormal.add(key)
     if key in by_id:retain(by_id[key],'abnormal QP / invalid or nonfinite candidate / primal violation')
     else:no_snapshot.append(dict(controlStep=key[0],localIteration=key[1],reason='abnormal attempt had no saved snapshot before cleanup'))
   held={int(f['controlStep']) for f in frames if truth(f['heldPreviousCommand'])}
   endpoints={int(frames[0]['controlStep'])}|{int(f['controlStep']) for f in frames[-3:]}
   for step in held|endpoints:
    for r in by_frame[step]:retain(r,'held command' if step in held else 'first or last three control frames')
   for field in [x for x in attempts[0] if x.startswith('nonlinearMinimum_')]:
    available=[a for a in attempts if finite(a.get(field))]
    if available:
     worst=min(available,key=lambda a:float(a[field]));key=(int(worst['controlStep']),int(worst['localIteration']))
     if key in by_id:retain(by_id[key],'worst nonlinear constraint group: '+field)
   # Preserve first/last examples needed by the unchanged default replay selector.
   for flag in [-2,0,1]:
    matches=[r for r in records if (r['exitflag']>0 if flag==1 else r['exitflag']==flag)]
    if matches:retain(matches[0],'default replay representative')
   initial=[r for r in records if r['localIteration']==0]
   if initial:retain(initial[-1],'last initial QP replay representative')
   case_reports.append(dict(case=folder.name,complete=True,abnormalAttempts=len(abnormal),heldFrames=len(held),preexistingMissingSnapshots=no_snapshot))
  for r in records:
   if not finite(r.get('exitflag')) or r['exitflag']<=0:retain(r,'abnormal indexed exitflag')
   if not finite(r.get('scaledInequalityViolation')):retain(r,'nonfinite or missing primal diagnostic')
   if finite(r.get('nonlinearFutureMinimum')) and r['nonlinearFutureMinimum']<0:retain(r,'negative nonlinear future residual')
   if r.get('boxCertificateRows') or r.get('exactZeroNegativeRows'):retain(r,'infeasibility certificate')
  top=sorted([r for r in records if finite(r.get('scaledApproximationError'))],key=lambda r:(-r['scaledApproximationError'],r['controlStep'],r['localIteration']))[:3]
  for r in top:retain(r,'top three approximation-error attempts')
  # Capture references embedded by preparation/replay scripts and MAT snapshots.
  # Existing replay representative selection was retained above; whole chunks
  # retain every local attempt at each selected control instant.
  original=OUT/'original_qp_indexes'/folder.name/'qp_snapshot_index.jsonl';original.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(index,original)
  retained_records=[]
  for n,r in enumerate(records,1):
   if r['file'].replace('\\','/') in reasons:retained_records.append(dict(**r,originalIndex=n,retentionReasons=sorted(reasons[r['file'].replace('\\','/')])) )
  write(OUT/'retained_indexes'/f'{folder.name}.json',retained_records)
  for p in sorted((folder/'qp_snapshots').glob('*.mat')):
   rel=p.relative_to(folder).as_posix();meta=known.get(str(p));assert meta and meta['bytes']==p.stat().st_size,str(p)
   item=dict(path=str(p),bytes=meta['bytes'],sha256=meta['sha256'],mtimeNs=p.stat().st_mtime_ns,case=folder.name)
   if rel not in paths or rel in reasons:
    item['reasons']=sorted(reasons.get(rel,{'unindexed snapshot: preserve'}));kept.append(item)
   else:
    item.update(reason='ordinary QP chunk; no anomaly or selected evidence; full raw trajectory retained',replacement=str(folder/'rawRun.mat'),category='ordinary_qp_chunk');candidates.append(item)
 write(OUT/'qp_candidates.json',candidates);write(OUT/'qp_retained.json',kept);write(OUT/'qp_coverage_before.json',case_reports)
 print('QP_SELECTION',len(candidates),'delete candidates',sum(x['bytes'] for x in candidates)/2**30,'GiB;',len(kept),'retained',flush=True)
if __name__=='__main__':main()
