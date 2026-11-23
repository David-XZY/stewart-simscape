import subprocess,zipfile,hashlib,shutil,os,stat
from pathlib import Path,PurePosixPath
from collections import defaultdict
from prepare_cleanup import ROOT,OUT,V8,EVIDENCE,read,write,digest,regular
SEVEN=Path(r'C:\Program Files\7-Zip\7z.exe')
def archive(p):return p.suffix.lower() in ['.7z','.zip','.rar','.tar','.gz','.bz2','.xz']
def disposable(p):
 parts=[s.lower() for s in p.relative_to(ROOT).parts]
 return ('archive_verification' in parts or 'delivery' in parts or 'development_runtime' in parts or
         (parts[0]=='.codex_tmp' and any('extraction' in s or 'extract_check' in s for s in parts)))
def cache(p):return '__pycache__' in p.parts or '.pytest_cache' in p.parts or p.suffix.lower()=='.pyc'
def stream_hash(stream):
 h=hashlib.sha256()
 for block in iter(lambda:stream.read(4*1024*1024),b''):h.update(block)
 return h.hexdigest()
def main():
 inventory=read(OUT/'inventory_before.json')['files'];protected={x['path'] for x in read(OUT/'protected_before.json')['files']}
 inventory_by={x['path']:x for x in inventory};pool=defaultdict(list);hashes={};candidates=[];retained=[]
 def h(p):
  key=str(p)
  if key not in hashes:hashes[key]=digest(p)
  return hashes[key]
 def entry(p,reason,replacement,category):
  s=p.stat();return dict(path=str(p),bytes=s.st_size,mtimeNs=s.st_mtime_ns,sha256=h(p),reason=reason,replacement=str(replacement),category=category)
 def rank(p):
  parts=p.relative_to(ROOT).parts;return (0 if p.is_relative_to(V8) else 1 if parts[0]=='stewart-simscape' else 2 if 'v7-estimator' in parts[0] else 3,len(parts),str(p))
 for item in inventory:
  p=Path(item['path'])
  if not disposable(p) and not cache(p) and not archive(p) and 'qp_snapshots' not in p.parts and not p.name.endswith('.incomplete'):
   pool[(p.name.lower(),item['bytes'])].append(p)
 for choices in pool.values():choices.sort(key=rank)
 def replacement(name,size,sha,exclude=None):
  for p in pool.get((Path(name).name.lower(),size),[]):
   if p!=exclude and h(p)==sha:return p
  return None
 for item in ([] if (OUT/'duplicate_candidates.json').exists() else inventory):
  # The candidate file is an immutable preparation checkpoint; no deletion
  # has happened yet when resuming archive inspection.
  p=Path(item['path'])
  if str(p) in protected:continue
  if cache(p):candidates.append(entry(p,'rebuildable Python/test cache','source files in same project','cache'));continue
  if p.name.endswith('.7z.incomplete') and p.parent==ROOT:
   candidates.append(entry(p,'explicitly incomplete archive; completed delivery retained',ROOT/'deliveries','incomplete_archive'));continue
  if disposable(p):
   found=replacement(p.name,item['bytes'],h(p),p)
   if found:candidates.append(entry(p,'byte-identical staging/verification copy',found,'duplicate_staging_file'))
   else:retained.append(dict(path=str(p),bytes=item['bytes'],reason='staging-looking path has no proven identical retained copy'))
 if (OUT/'duplicate_candidates.json').exists():candidates=read(OUT/'duplicate_candidates.json')
 else:write(OUT/'duplicate_candidates.json',candidates);write(OUT/'unique_staging_retained.json',retained)
 print('STAGING_AND_CACHE_CANDIDATES',len(candidates),sum(x['bytes'] for x in candidates)/2**30,flush=True)
 archive_reports=[];archive_candidates=[];temp_candidates=[]
 # All archive locations are inventoried. Only obsolete root Stewart/MPC
 # deliveries have an established successor; dependency fixtures and distinct
 # research deliveries do not become obsolete merely because they are old.
 all_archives=[Path(x['path']) for x in inventory if archive(Path(x['path']))]
 for p in all_archives:
  eligible=p.parent==ROOT and (p.name.lower().startswith('stewart') or p.name.startswith('MPC_'))
  if not eligible:
   archive_reports.append(dict(path=str(p),action='retain',reason='latest delivery, independent research delivery, or dependency/input archive; no established successor'));continue
  print('CHECK_ARCHIVE',p.name,flush=True)
  previous=OUT/'archive_member_maps'/f'{p.stem}.json'
  if previous.exists():
   saved=read(previous);assert saved['archiveSHA256']==h(p) and saved['integrityPassed']
   retired=saved['retired'];members=saved['members'];unique_bytes=sum(x['bytes'] for x in members if x['kind']!='existing_exact_copy')
   if retired:
    archive_candidates.append(entry(p,'obsolete delivery: every member has a verified retained copy',previous,'superseded_archive'))
    for suffix in ['.sha256','.sha256.txt','.verification.json']:
     for side in [Path(str(p)+suffix),p.with_suffix(suffix)]:
      if side.exists() and side!=p and str(side) in inventory_by and not any(x['path']==str(side) for x in archive_candidates):archive_candidates.append(entry(side,'sidecar of verified superseded archive',previous,'retired_archive_sidecar'))
   archive_reports.append(dict(path=str(p),action='delete' if retired else 'retain',archiveBytes=p.stat().st_size,duplicateUncompressedBytes=sum(x['bytes'] for x in members if x['kind']=='existing_exact_copy'),uniqueUncompressedBytes=unique_bytes,reason='all members preserved; net space released' if retired else 'unique content lacks an existing replacement; extraction would not save space',members=len(members)))
   continue
  members=[];temporary=None;unique=[];duplicate_bytes=0
  if p.suffix.lower()=='.7z':
   listed=subprocess.run([str(SEVEN),'l','-slt','-sccUTF-8',str(p)],capture_output=True,check=True).stdout.decode('utf8')
   listing=listed.split('----------',1)[1].replace('\r\n','\n')
   for block in listing.strip().split('\n\n'):
    properties=dict(tuple(part.strip() for part in line.split(' = ',1)) for line in block.splitlines() if ' = ' in line)
    name=properties.get('Path','').replace('\\','/')
    if not name:continue
    relative=PurePosixPath(name);assert not relative.is_absolute() and '..' not in relative.parts and ':' not in name,(p,name)
   temporary=OUT/'archive_verification_temp'/p.stem;assert not temporary.exists();temporary.mkdir(parents=True)
   result=subprocess.run([str(SEVEN),'x','-y','-sccUTF-8','-o'+str(temporary),str(p)],capture_output=True)
   (OUT/(p.stem+'_7z_verification.log')).write_bytes(result.stdout+result.stderr)
   assert result.returncode==0,(p,result.returncode)
   for file in temporary.rglob('*'):
    assert regular(file),file
    if file.is_file():members.append((file.relative_to(temporary).as_posix(),file.stat().st_size,h(file),file))
  elif p.suffix.lower()=='.zip':
   with zipfile.ZipFile(p) as z:
    for info in z.infolist():
     if info.is_dir():continue
     name=info.filename.replace('\\','/');relative=PurePosixPath(name)
     assert not relative.is_absolute() and '..' not in relative.parts and ':' not in name,(p,name)
     with z.open(info) as f:sha=stream_hash(f)
     members.append((name,info.file_size,sha,None))
  else:
   archive_reports.append(dict(path=str(p),action='retain',reason='unsupported archive type'));continue
  mappings=[]
  for name,size,sha,file in members:
   found=replacement(name,size,sha)
   if found:duplicate_bytes+=size;mappings.append(dict(member=name,bytes=size,sha256=sha,replacement=str(found),kind='existing_exact_copy'))
   else:unique.append((name,size,sha,file))
  unique_bytes=sum(x[1] for x in unique)
  # Extracting novel content must not consume more than deleting the package
  # releases. Otherwise retain the intact archive and its compact storage.
  retire=unique_bytes<p.stat().st_size and duplicate_bytes>0
  if retire:
   target=OUT/'retained_archive_content'/p.stem
   z=zipfile.ZipFile(p) if p.suffix.lower()=='.zip' else None
   try:
    for name,size,sha,file in unique:
     destination=target/Path(name);destination.parent.mkdir(parents=True,exist_ok=True);assert not destination.exists()
     if file:shutil.copy2(file,destination)
     else:
      with z.open(name) as src,destination.open('wb') as dst:shutil.copyfileobj(src,dst,4*1024*1024)
     assert destination.stat().st_size==size and digest(destination)==sha
     mappings.append(dict(member=name,bytes=size,sha256=sha,replacement=str(destination),kind='unique_content_preserved'))
   finally:
    if z:z.close()
   archive_candidates.append(entry(p,'obsolete delivery: every member has a verified retained copy',OUT/'archive_member_maps'/f'{p.stem}.json','superseded_archive'))
   # Sidecars belong only to this archive or its exact stem.
   for suffix in ['.sha256','.sha256.txt','.verification.json']:
    for side in [Path(str(p)+suffix),p.with_suffix(suffix)]:
     if side.exists() and side!=p and str(side) in inventory_by and not any(x['path']==str(side) for x in archive_candidates):
      archive_candidates.append(entry(side,'sidecar of verified superseded archive',OUT/'archive_member_maps'/f'{p.stem}.json','retired_archive_sidecar'))
  else:
   mappings.extend(dict(member=name,bytes=size,sha256=sha,kind='unique_content_remains_in_retained_archive') for name,size,sha,file in unique)
  write(OUT/'archive_member_maps'/f'{p.stem}.json',dict(archive=str(p),archiveSHA256=h(p),integrityPassed=True,retired=retire,members=mappings))
  archive_reports.append(dict(path=str(p),action='delete' if retire else 'retain',archiveBytes=p.stat().st_size,duplicateUncompressedBytes=duplicate_bytes,uniqueUncompressedBytes=unique_bytes,
       reason='all members preserved; net space released' if retire else 'unique content lacks an existing replacement; extraction would not save space',members=len(members)))
  if temporary:
   for _,_,_,file in members:temp_candidates.append(entry(file,'temporary archive verification extraction; archive or canonical content retained',p,'verification_temporary'))
  print('ARCHIVE_DECISION',p.name,'delete' if retire else 'retain','uniqueBytes',unique_bytes,flush=True)
 write(OUT/'archive_candidates.json',archive_candidates);write(OUT/'verification_temp_candidates.json',temp_candidates);write(OUT/'archive_decisions.json',archive_reports)
 print('ARCHIVE_PASS',len(archive_candidates),'archive/sidecar candidates; preserved unique bytes',sum(x.get('uniqueUncompressedBytes',0) for x in archive_reports if x['action']=='delete'),flush=True)
if __name__=='__main__':main()
