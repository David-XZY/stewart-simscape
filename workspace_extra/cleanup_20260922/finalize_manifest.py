from collections import Counter
import shutil,datetime
from prepare_cleanup import *
def main():
 candidates={}
 for name in ['qp_candidates.json','duplicate_candidates.json','archive_candidates.json','verification_temp_candidates.json']:
  for item in read(OUT/name):
   if item['path'] in candidates:assert candidates[item['path']]['sha256']==item['sha256']
   else:candidates[item['path']]=item
 protected=read(OUT/'protected_before.json');protected_paths={x['path'] for x in protected['files']}|{protected['archive']['archive']}
 protected_paths|={x['path'] for x in read(OUT/'qp_retained.json')}
 assert not (set(candidates)&protected_paths)
 canonical=[]
 for item in candidates.values():
  p=Path(item['path']).resolve();assert p.is_relative_to(ROOT) and p!=ROOT and p.is_file() and regular(p)
  assert p.stat().st_size==item['bytes'],p
  if item['category']=='duplicate_staging_file':
   q=Path(item['replacement']);assert str(q) not in candidates and q.exists() and digest(q)==item['sha256'],q
   canonical.append(dict(path=str(q),sha256=item['sha256'],bytes=item['bytes']))
 for record in read(OUT/'archive_decisions.json'):
  if record['action']!='delete':continue
  p=Path(record['path']);mapping=read(OUT/'archive_member_maps'/f'{p.stem}.json')
  assert mapping['integrityPassed'] and mapping['retired']
  for member in mapping['members']:
   q=Path(member['replacement']);assert str(q) not in candidates and q.exists() and q.stat().st_size==member['bytes'] and digest(q)==member['sha256'],q
   canonical.append(dict(path=str(q),sha256=member['sha256'],bytes=member['bytes']))
 # Compare retained full matrices against the package-time immutable hashes.
 for item in read(OUT/'qp_retained.json'):assert digest(Path(item['path']))==item['sha256'],item['path']
 unique={x['path']:x for x in canonical};write(OUT/'canonical_replacements.json',list(unique.values()))
 groups=Counter();counts=Counter()
 for x in candidates.values():groups[x['category']]+=x['bytes'];counts[x['category']]+=1
 manifest=dict(createdAt=datetime.datetime.now().astimezone().isoformat(),root=str(ROOT),diskFreeBeforeDeletion=shutil.disk_usage(ROOT).free,
   approval='User explicitly requested implementation of the agreed evidence-preserving cleanup plan.',
   categoryBytes=dict(groups),categoryCounts=dict(counts),files=list(candidates.values()),protectedPaths=sorted(protected_paths))
 write(OUT/'deletion_manifest.json',manifest)
 write(OUT/'preflight_passed.json',dict(passed=True,manifestSHA256=digest(OUT/'deletion_manifest.json'),latestArchiveUnchanged=digest(Path(protected['archive']['archive']))==protected['archive']['sha256'],canonicalFilesVerified=len(unique),retainedQpFilesVerified=len(read(OUT/'qp_retained.json'))))
 print('DELETION_MANIFEST_READY',len(candidates),dict(groups),flush=True)
if __name__=='__main__':main()
