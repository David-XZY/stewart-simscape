"""Stop only this cleanup's queue, after its active compact children finish."""
import ctypes,json,subprocess,time
from pathlib import Path
from inventory import OUT,ROOT,allocated
from compress import sha

k=ctypes.WinDLL('kernel32',use_last_error=True);nt=ctypes.WinDLL('ntdll')
k.OpenProcess.argtypes=[ctypes.c_uint32,ctypes.c_int,ctypes.c_uint32];k.OpenProcess.restype=ctypes.c_void_p
k.CloseHandle.argtypes=[ctypes.c_void_p];k.WaitForSingleObject.argtypes=[ctypes.c_void_p,ctypes.c_uint32]
k.TerminateProcess.argtypes=[ctypes.c_void_p,ctypes.c_uint32]
nt.NtSuspendProcess.argtypes=[ctypes.c_void_p];nt.NtResumeProcess.argtypes=[ctypes.c_void_p]
shell=ctypes.WinDLL('shell32');shell.CommandLineToArgvW.argtypes=[ctypes.c_wchar_p,ctypes.POINTER(ctypes.c_int)];shell.CommandLineToArgvW.restype=ctypes.POINTER(ctypes.c_wchar_p)
k.LocalFree.argtypes=[ctypes.c_void_p]

def processes():
    command="Get-CimInstance Win32_Process | Where-Object { $_.Name -in @('python.exe','compact.exe') } | Select-Object ProcessId,ParentProcessId,Name,CommandLine | ConvertTo-Json -Compress"
    result=subprocess.run(['powershell.exe','-NoProfile','-Command',command],capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW,check=True)
    data=json.loads(result.stdout.decode('utf-8-sig'));return data if isinstance(data,list) else [data]

def args(text):
    count=ctypes.c_int();pointer=shell.CommandLineToArgvW(text,ctypes.byref(count))
    try:return [pointer[i] for i in range(count.value)]
    finally:k.LocalFree(pointer)

def main():
    jobs=[];handles=[];children=[];receipt=dict(reason='User requested a review list for obsolete material deletion instead of further compression.',jobs=[],active_files=[])
    for p in processes():
        command=(p.get('CommandLine') or '').replace('\\','/')
        if p['Name']=='python.exe' and 'cleanup_20261123/compact_lzx.py ' in command and ('--label lzx' in command or '--label additional_lzx' in command):jobs.append(p)
    try:
        for p in jobs:
            h=k.OpenProcess(0x1801,False,p['ProcessId'])
            if not h:continue
            if nt.NtSuspendProcess(h)!=0:k.CloseHandle(h);raise RuntimeError('Cannot pause own compression queue')
            handles.append((h,p));receipt['jobs'].append(p)
        pids={p['ProcessId'] for h,p in handles}
        for p in processes():
            if p['Name']!='compact.exe' or p['ParentProcessId'] not in pids:continue
            file=Path(args(p['CommandLine'])[-1]).resolve()
            if not file.is_relative_to(ROOT):raise RuntimeError('Owned child target outside workspace')
            ch=k.OpenProcess(0x100000,False,p['ProcessId'])
            record=dict(path=file.relative_to(ROOT).as_posix(),pid=p['ProcessId'],sha256_at_pause=sha(file),allocated_at_pause=allocated(file))
            children.append((ch,file,record))
        print('OWN_QUEUES_PAUSED',len(handles),'finishing_files',len(children),flush=True)
        deadline=time.monotonic()+120
        for ch,file,record in children:
            if ch:
                while k.WaitForSingleObject(ch,0)!=0:
                    if time.monotonic()>deadline:raise TimeoutError('Active file operation still finishing')
                    time.sleep(.2)
                k.CloseHandle(ch)
            record.update(sha256_after_finish=sha(file),allocated_after_finish=allocated(file))
            if record['sha256_at_pause']!=record['sha256_after_finish']:raise RuntimeError('Active file changed during finish')
            receipt['active_files'].append(record)
        for h,p in handles:
            if not k.TerminateProcess(h,0):raise OSError(ctypes.get_last_error())
            k.CloseHandle(h)
        handles=[];receipt['status']='stopped_after_active_file_operations_finished';receipt['created']=time.strftime('%Y-%m-%d %H:%M:%S')
        (OUT/'compression_paused_for_review.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2),encoding='utf-8')
        print('OWN_COMPRESSION_STOPPED',len(jobs),flush=True)
    finally:
        for h,p in handles:nt.NtResumeProcess(h);k.CloseHandle(h)

if __name__=='__main__':main()
