"""Attach to the existing MATLAB worker; tolerate transient Windows heartbeat locks."""
import json,os,subprocess,sys,time,traceback
import psutil
from prepare import ROOT,HERE,OUT,read,write
import runner
import report_resume_carry as campaign

def heartbeat():
    try:return read(OUT/'formal/heartbeat.json')
    except (PermissionError,FileNotFoundError,json.JSONDecodeError):return None

def supervise(process,started):
    while True:
        try:return process.wait(timeout=.25)
        except psutil.TimeoutExpired:pass
        except psutil.NoSuchProcess:return None
        h=heartbeat()
        if h and h['unixTime']>=started and h['stage']=='online' and time.time()-h['unixTime']>h['B']+30:
            write(OUT/'formal/watchdog_failure.json',dict(heartbeat=h,pid=process.pid,observedUnix=time.time()))
            subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True)
            raise RuntimeError('Online call exceeded B+30s; incomplete attempt retained')

def safe_matlab(expression,phase,watchdog=False):
    env=os.environ.copy();env['STEWART_DEPENDENCY_ROOT']=str(ROOT.parent/'stewart-simscape-abfd')
    cmd=[os.environ.get('PG_MATLAB','D:/matlab2019a/bin/matlab.exe'),'-wait','-singleCompThread','-batch',
      "addpath('chapter3/payload_generalization','analysis/payload_pair2_nofallback'); "+expression]
    with (OUT/phase/'matlab.log').open('a',encoding='utf-8') as log:
        started=time.time();p=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        code=supervise(psutil.Process(p.pid),started);p.wait()
        if code not in [0,None] or p.returncode not in [0,None]:raise RuntimeError(f'MATLAB exit {code or p.returncode}')

def finish():
    p=read(OUT/'protocol_carry.json');result=campaign.audit(p)
    from report_summarize_carry import summarize
    summarize();runner.status('carry_completed',**result)
    subprocess.run([sys.executable,str(HERE/'report_figures_carry.py')],check=True)

def main():
    owned=[]
    for p in psutil.process_iter(['pid','name','cmdline']):
        cmd=p.info['cmdline'] or []
        if p.info['name'].lower()=='matlab.exe' and '-wait' in cmd and 'p3CarryBatch' in ' '.join(cmd):owned.append(p)
    assert len(owned)<=1,'Multiple planning workers; refusing to start another'
    previous=read(OUT/'status.json');write(OUT/'monitor_recovery_previous_status.json',previous)
    write(OUT/'recovery_status.json',dict(active=True,mode='reattached_race_safe_monitor',pid=os.getpid(),workerPid=owned[0].pid if owned else None))
    runner.status('carry_running',logicalCells=10950,monitorReattached=True,workerPid=owned[0].pid if owned else None)
    runner.matlab=safe_matlab
    if owned:
        code=supervise(owned[0],owned[0].create_time())
        write(OUT/'attached_worker_exit.json',dict(exit=code,unixTime=time.time()))
        if code in [0,None] and len(list((OUT/'carry/evaluations').glob('*.json')))==10950:
            finish();return
        assert code in [3221226356,-1073740940,None],f'Unexpected worker exit {code}; investigate before retry'
    for attempt in range(1,6):
        before=len(list((OUT/'carry/evaluations').glob('*.json')))
        try:campaign.main();return
        except RuntimeError as e:
            after=len(list((OUT/'carry/evaluations').glob('*.json')))
            write(OUT/f'native_retry_{attempt}.json',dict(error=str(e),before=before,after=after))
            if '3221226356' not in str(e) and '-1073740940' not in str(e):raise
            if after<=before:raise
    raise RuntimeError('Native crash retry limit reached')

if __name__=='__main__':
    import msvcrt
    lock=(OUT/'worker.lock').open('a+b');lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    try:
        main();write(OUT/'recovery_status.json',dict(active=False,completed=True,mode='race_safe_monitor'))
    except BaseException:
        runner.status('failed',error=traceback.format_exc());write(OUT/'recovery_status.json',dict(active=False,completed=False));raise
    finally:lock.close()
