"""Select stable, explicitly scoped candidates; preserve active research."""
import collections,json,time
from pathlib import Path
from inventory import ROOT,OUT
from compress import sha

def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def write(p,v):Path(p).write_text(json.dumps(v,ensure_ascii=False,indent=2),encoding='utf-8')

def main():
    rows=[json.loads(s) for s in (OUT/'inventory.jsonl').open(encoding='utf-8')];cutoff=(time.time()-24*3600)*1e9
    prefix='stewart_unified_nmpc/'
    excluded=[prefix+'ch5_adaptive/',prefix+'results/ch5_adaptive_20261123/',prefix+'cache/ch5_adaptive_20261123/',
              prefix+'results/ch4_noopt_20261121/',prefix+'cache/ch4_noopt_20261121/',prefix+'cache/long_payload_v2_20261109/python/',
              prefix+'cache/nmpc_rti_20261115/vendor_simd/']
    def allowed(r):
        p=r['path'];parts=p.split('/')
        return r['mtime_ns']<cutoff and not any(p.startswith(v) for v in excluded) and not any(s in parts for s in ('.git','third_party','.venv','venv','site-packages','node_modules','vendor_simd'))
    # Research source and result files remain at their original paths. No raw
    # traces, failed runs, logs, solver libraries, or models are deleted.
    compact=[r for r in rows if allowed(r) and r['bytes']>=1024**2 and Path(r['path']).suffix.lower() in ('.c','.csv','.json','.log','.txt','.mat','.obj')]
    # Cold CasADi object caches are explicitly rebuilt when absent (verified
    # in compile_casadi.py / compile_repair_casadi.py); retain their receipts.
    removable=[r for r in rows if allowed(r) and r['path'].startswith(prefix+'cache/') and Path(r['path']).suffix=='.obj'
               and ('/compiler_objects/' in r['path'] or Path(r['path']).name.startswith('tmp_casadi_compiler_shell'))]
    remove_set={r['path'] for r in removable};compact=[r for r in compact if r['path'] not in remove_set]
    # Windows DirEntry.stat omits inode/device; acquire real identities only
    # for selected candidates, without treating a changed file as eligible.
    def identities(items):
        result=[]
        for r in items:
            s=(ROOT/r['path']).stat()
            if (s.st_size,s.st_mtime_ns)!=(r['bytes'],r['mtime_ns']):continue
            result.append(dict(r,inode=s.st_ino,device=s.st_dev))
        return result
    compact=identities(compact);removable=identities(removable)
    write(OUT/'compression_candidates.json',compact);write(OUT/'object_candidates.json',removable)
    samples=[]
    for suffix,sub in [('.c',prefix+'cache/ch3_ihsid_20261117/'),('.csv','stewart-simscape-ch5-integration/'),('.mat',prefix+'results/ch4_tracking_20261120/')]:
        matching=[r for r in compact if r['path'].startswith(sub) and r['path'].endswith(suffix)]
        if not matching:matching=[r for r in compact if r['path'].endswith(suffix)]
        if matching:samples.append(sorted(matching,key=lambda r:abs(r['bytes']-64*1024**2))[0])
    write(OUT/'compression_probe.json',samples)
    old=read(ROOT/prefix/'results/ch5_adaptive_20261123/baseline.json')['files'];checks={};mismatch=[]
    for name,expected in old.items():
        actual=sha(ROOT/prefix/name);checks[prefix+name]=actual
        if expected!=actual:mismatch.append(name)
    write(OUT/'protected_before.json',dict(files=checks,mismatch_to_main_baseline=mismatch))
    if mismatch:raise RuntimeError('Main protected baseline already differs; inspect before cleanup')
    print(json.dumps(dict(compression_files=len(compact),compression_GiB=sum(r['bytes'] for r in compact)/2**30,
                          removable_files=len(removable),removable_GiB=sum(r['bytes'] for r in removable)/2**30,protected_files=len(checks),probe=[r['path'] for r in samples]),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
