"""Only immutable generated C/object files; keep live binaries and all data."""
import json,time
from pathlib import Path
from inventory import ROOT,OUT

def main():
    rows=[json.loads(s) for s in (OUT/'inventory.jsonl').open(encoding='utf-8')];selected=[]
    loaded={str(Path(v)).lower() for v in json.loads((OUT/'loaded_modules_before.json').read_text(encoding='utf-8-sig'))}
    retained={json.loads(s)['path'] for s in (OUT/'deleted_objects.jsonl').open(encoding='utf-8') if json.loads(s)['status']=='retained_temp_without_verified_duplicate'}
    for r in rows:
        name=r['path'];p=ROOT/name
        old_library=(p.suffix=='.dll' and str(p).lower() not in loaded and not any(part in name.split('/') for part in ('.git','third_party','site-packages','.venv','python','vendor_simd'))
                     and not name.startswith(('stewart_unified_nmpc/cache/ch4_noopt_20261121/','stewart_unified_nmpc/results/ch4_noopt_20261121/','stewart_unified_nmpc/cache/ch5_adaptive_20261123/','stewart_unified_nmpc/results/ch5_adaptive_20261123/')))
        if not ((name.startswith('stewart_unified_nmpc/cache/ch4_noopt_20261121/') and p.suffix in ('.c','.obj')) or name in retained or old_library):continue
        if r['bytes']<1024**2 or r['mtime_ns']>(time.time()-86400)*1e9:continue
        s=p.stat()
        if (s.st_size,s.st_mtime_ns)!=(r['bytes'],r['mtime_ns']):continue
        selected.append(dict(r,inode=s.st_ino,device=s.st_dev))
    (OUT/'additional_candidates.json').write_text(json.dumps(selected,ensure_ascii=False,indent=2),encoding='utf-8')
    print('ADDITIONAL_IN_PLACE_ONLY',len(selected),'GiB',sum(r['bytes'] for r in selected)/2**30)

if __name__=='__main__':main()
