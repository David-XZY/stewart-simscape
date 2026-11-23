"""File-by-file Windows LZX compression; paths and content remain readable.

Only fixed, cold candidates are processed. Each file is hashed before/after;
changing, replaced, missing, or locked files are skipped. No directory flags
are changed, and no OS-wide compression policy is used.
"""
import argparse,concurrent.futures,ctypes,json,os,subprocess,time,zlib
from pathlib import Path
from compress import sha,safe_path,kernel
from inventory import ROOT,OUT,allocated

def sample_ratio(path,size):
    blocks=[]
    with path.open('rb') as f:
        for frac in (.13,.37,.61,.83):
            f.seek(int(max(0,size-262144)*frac));blocks.append(f.read(262144))
    return sum(len(zlib.compress(b,1)) for b in blocks)/max(1,sum(map(len,blocks)))

def one(row):
    name=row['path'];p=safe_path(name);s=p.stat();start=time.perf_counter()
    if (s.st_size,s.st_mtime_ns,s.st_ino)!=(row['bytes'],row['mtime_ns'],row['inode']):return dict(path=name,status='skipped_changed')
    if p.suffix=='.mat':
        ratio=sample_ratio(p,s.st_size)
        if ratio>.90:return dict(path=name,status='skipped_already_dense',sample_compression_ratio=ratio)
    digest=sha(p);before=allocated(p)
    args=['compact.exe','/C','/EXE:LZX','/F','/I','/Q','/A',str(p)]
    run=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
    if run.returncode:return dict(path=name,status='skipped_compact_error',returncode=run.returncode,output=run.stdout.decode(errors='replace'))
    after=allocated(p);current=p.stat();new_digest=sha(p)
    if digest!=new_digest or (s.st_size,s.st_mtime_ns,s.st_ino)!=(current.st_size,current.st_mtime_ns,current.st_ino):
        return dict(path=name,status='concurrent_change_detected',sha_before=digest,sha_after=new_digest,before_allocated=before,after_allocated=after)
    return dict(path=name,status='lzx_verified',sha256=digest,logical_bytes=s.st_size,before_allocated=before,after_allocated=after,saved_bytes=before-after,seconds=time.perf_counter()-start,command=args)

def main(manifest,label,workers):
    kernel.SetPriorityClass(kernel.GetCurrentProcess(),0x00004000)
    rows=json.loads(Path(manifest).read_text(encoding='utf-8'));log=OUT/(label+'_actions.jsonl');done={};saved=0
    if log.exists():
        for line in log.open(encoding='utf-8'):
            r=json.loads(line);done[r['path']]=r
    saved=sum(r.get('saved_bytes',0) for r in done.values());rows=[r for r in rows if r['path'] not in done]
    rows.sort(key=lambda r:(Path(r['path']).suffix=='.mat',-r['bytes']))
    completed=len(done);last=time.monotonic()
    with log.open('a',encoding='utf-8') as stream,concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        todo=iter(rows);pending={}
        for r in [next(todo,None) for _ in range(workers)]:
            if r is not None:pending[pool.submit(one,r)]=r
        while pending:
            finished,_=concurrent.futures.wait(pending,timeout=5,return_when=concurrent.futures.FIRST_COMPLETED)
            for job in finished:
                row=pending.pop(job)
                try:r=job.result()
                except OSError as e:r=dict(path=row['path'],status='skipped_os_error',error=str(e))
                stream.write(json.dumps(r,ensure_ascii=False)+'\n');stream.flush();completed+=1;saved+=r.get('saved_bytes',0)
                nxt=next(todo,None)
                if nxt is not None:pending[pool.submit(one,nxt)]=nxt
            if time.monotonic()-last>20:
                print(json.dumps(dict(processed=completed,saved_GiB=round(saved/2**30,3),pending=[r['path'] for r in pending.values()]),ensure_ascii=False),flush=True);last=time.monotonic()
    print(json.dumps(dict(completed=True,processed=completed,saved_GiB=saved/2**30,label=label)),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('manifest');p.add_argument('--label',required=True);p.add_argument('--workers',type=int,default=2);a=p.parse_args();main(a.manifest,a.label,a.workers)
