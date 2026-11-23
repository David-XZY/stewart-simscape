"""Delete only cold, regenerable CasADi object-cache files on an exact list."""
import json,re,time
from pathlib import Path
from inventory import ROOT,OUT,allocated
from compress import safe_path,sha,kernel

def main():
    kernel.SetPriorityClass(kernel.GetCurrentProcess(),0x00004000)
    rows=json.loads((OUT/'object_candidates.json').read_text(encoding='utf-8'));protected=json.loads((OUT/'protected_before.json').read_text(encoding='utf-8'))['files']
    previous=json.loads((ROOT/'cleanup_20261117/protected_before.json').read_text(encoding='utf-8'));removed=0;count=0;verified_hashes=set()
    log=OUT/'deleted_objects.jsonl'
    if log.exists():
        for line in log.open(encoding='utf-8'):
            r=json.loads(line)
            if r['status']=='removed_regenerable_object':removed+=r['allocated_bytes'];count+=1;verified_hashes.add(r['sha256'])
    # Verify cache entries before accepting temporary copies as duplicates.
    rows.sort(key=lambda r:('/compiler_objects/' not in r['path'],r['path']))
    with log.open('a',encoding='utf-8') as stream:
        for r in rows:
            name=r['path'];p=ROOT/name
            if not p.exists():continue
            if name in protected or name in previous:continue
            if not name.startswith('stewart_unified_nmpc/cache/') or '/ch5_adaptive_' in name or '/ch4_noopt_' in name:raise ValueError('Protected or out-of-scope object')
            is_cache='/compiler_objects/' in name and re.fullmatch('[0-9a-f]{64}\\.obj',p.name)
            is_temp=re.fullmatch('tmp_casadi_compiler_shell[0-9a-zA-Z_]+\\.obj',p.name)
            if not (is_cache or is_temp):raise ValueError('Unexpected object filename')
            p=safe_path(name);s=p.stat()
            if (s.st_size,s.st_mtime_ns,s.st_ino)!=(r['bytes'],r['mtime_ns'],r['inode']):continue
            digest=sha(p)
            if is_cache:
                meta=p.with_suffix('.json');evidence=json.loads(meta.read_text(encoding='utf-8-sig'))
                if digest!=evidence.get('objectSHA256'):raise RuntimeError('Object cache integrity differs: '+name)
                verified_hashes.add(digest)
            elif digest not in verified_hashes:
                stream.write(json.dumps(dict(path=name,status='retained_temp_without_verified_duplicate',sha256=digest))+'\n');stream.flush();continue
            final=p.stat()
            if (final.st_size,final.st_mtime_ns,final.st_ino)!=(s.st_size,s.st_mtime_ns,s.st_ino):continue
            size=allocated(p);p.unlink()
            if p.exists():raise RuntimeError('File still exists after removal')
            record=dict(path=name,status='removed_regenerable_object',sha256=digest,bytes=s.st_size,allocated_bytes=size,
                        reason='Cold CasADi object cache; compiler wrappers rebuild absent objects. Generated C, loaded DLLs, compiler receipts and logs retained.')
            stream.write(json.dumps(record,ensure_ascii=False)+'\n');stream.flush();removed+=size;count+=1
            if count%100==0:print('OBJECTS_REMOVED',count,'GiB',round(removed/2**30,3),flush=True)
    print('OBJECT_CLEANUP_COMPLETE',count,'GiB',round(removed/2**30,6),flush=True)

if __name__=='__main__':main()
