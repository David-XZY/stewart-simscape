"""In-place NTFS compression with exclusive-write lock and byte hash checks.

No paths, payload bytes, or modification times are changed. Newly created
files do not inherit compression because no directory attributes are changed.
"""
import argparse,ctypes,hashlib,json,os,stat,time
from pathlib import Path
from inventory import ROOT,OUT,allocated

kernel=ctypes.WinDLL('kernel32',use_last_error=True)
kernel.CreateFileW.argtypes=[ctypes.c_wchar_p,ctypes.c_uint32,ctypes.c_uint32,ctypes.c_void_p,ctypes.c_uint32,ctypes.c_uint32,ctypes.c_void_p]
kernel.CreateFileW.restype=ctypes.c_void_p
kernel.CloseHandle.argtypes=[ctypes.c_void_p]
kernel.GetCurrentProcess.restype=ctypes.c_void_p
kernel.SetPriorityClass.argtypes=[ctypes.c_void_p,ctypes.c_uint32]
kernel.DeviceIoControl.argtypes=[ctypes.c_void_p,ctypes.c_uint32,ctypes.c_void_p,ctypes.c_uint32,ctypes.c_void_p,ctypes.c_uint32,ctypes.POINTER(ctypes.c_uint32),ctypes.c_void_p]
kernel.DeviceIoControl.restype=ctypes.c_int

def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def safe_path(rel):
    p=ROOT/rel
    if not p.is_absolute() or not p.resolve().is_relative_to(ROOT) or p==ROOT:raise ValueError('Outside authorized workspace')
    for a in (p,*p.parents):
        if a==ROOT:break
        if a.lstat().st_file_attributes & stat.FILE_ATTRIBUTE_REPARSE_POINT:raise ValueError('Reparse point excluded')
    return p

def compress(row):
    p=safe_path(row['path']);before=p.stat()
    if (before.st_size,before.st_mtime_ns,before.st_ino)!=(row['bytes'],row['mtime_ns'],row['inode']):return dict(path=row['path'],status='skipped_changed')
    # Share reads only: no other writer can race compression or byte checking.
    handle=kernel.CreateFileW(str(p),0xC0000000,1,None,3,0x08000000,None)
    if handle in (None,ctypes.c_void_p(-1).value):return dict(path=row['path'],status='skipped_locked',error=ctypes.get_last_error())
    try:
        digest=sha(p);old_allocated=allocated(p);mode=ctypes.c_ushort(2);returned=ctypes.c_uint32();start=time.perf_counter()
        if not kernel.DeviceIoControl(handle,0x9C040,ctypes.byref(mode),2,None,0,ctypes.byref(returned),None):
            return dict(path=row['path'],status='skipped_compression_error',error=ctypes.get_last_error())
        new_digest=sha(p);after=p.stat();new_allocated=allocated(p)
        if digest!=new_digest or after.st_size!=before.st_size or after.st_mtime_ns!=before.st_mtime_ns:raise RuntimeError('File content/mtime changed: '+str(p))
        return dict(path=row['path'],status='compressed_verified',sha256=digest,logical_bytes=before.st_size,before_allocated=old_allocated,after_allocated=new_allocated,saved_bytes=old_allocated-new_allocated,seconds=time.perf_counter()-start)
    finally:kernel.CloseHandle(handle)

def run(manifest,label):
    # Keep cleanup CPU work below the running research process.
    kernel.SetPriorityClass(kernel.GetCurrentProcess(),0x00004000)
    rows=json.loads(Path(manifest).read_text(encoding='utf-8'));done=set();log=OUT/(label+'_actions.jsonl');saved=0;n=0
    if log.exists():
        for line in log.open(encoding='utf-8'):
            r=json.loads(line)
            if r['status']=='compressed_verified':done.add(r['path']);saved+=r['saved_bytes'];n+=1
    with log.open('a',encoding='utf-8') as f:
        for row in rows:
            if row['path'] in done:continue
            r=compress(row);f.write(json.dumps(r,ensure_ascii=False)+'\n');f.flush()
            if r['status']=='compressed_verified':saved+=r['saved_bytes'];n+=1
            if n%25==0 or r['status']!='compressed_verified':print(json.dumps(dict(count=n,saved_GiB=round(saved/2**30,3),last=r['path'],status=r['status']),ensure_ascii=False),flush=True)
    print(json.dumps(dict(completed=True,count=n,saved_GiB=saved/2**30,label=label)),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('manifest');p.add_argument('--label',required=True);a=p.parse_args();run(a.manifest,a.label)
