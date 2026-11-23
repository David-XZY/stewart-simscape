"""Operator-invoked recovery after a VERIFIED native crash/controller exit.

Does not alter the frozen numerical implementation or deadline. Explicit job
identities and a native error at the end of the worker log are required.
"""
import argparse
import base64
import datetime as dt
import json
from pathlib import Path
import re
import shutil
import subprocess


def powershell(source):
    source="$ErrorActionPreference='Stop'; "+source
    encoded=base64.b64encode(source.encode('utf-16-le')).decode('ascii')
    return subprocess.run(['powershell','-NoProfile','-NonInteractive','-EncodedCommand',encoded],
                          capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--failed-worker',type=int,choices=[1,2],required=True)
    p.add_argument('--failed-job',required=True)
    p.add_argument('--interrupted-job',required=True)
    p.add_argument('--incident',required=True)
    a=p.parse_args()
    for value in [a.failed_job,a.interrupted_job,a.incident]:
        assert re.fullmatch('[a-zA-Z0-9_]+',value),'Invalid identity.'
    project=Path(__file__).resolve().parents[1]; root=project/'results'/'study'/'frozen_v1'
    check=powershell("$ProgressPreference='SilentlyContinue'; $p=@(Get-CimInstance Win32_Process | "
                    "Where-Object { $_.Name -match 'python|MATLAB' -and $_.CommandLine -match "
                    "'sgWorker\\(|study[/\\\\](run_study|launch_supervisor).py' }); if($p.Count){exit 9}")
    assert check.returncode==0,'Study process still running or process inspection failed.'
    failed=read(root/f'active_{a.failed_worker}.json')
    other=read(root/f'active_{3-a.failed_worker}.json')
    assert failed['jobId']==a.failed_job and other['jobId']==a.interrupted_job,'Active jobs changed.'
    log=root/f'worker_{a.failed_worker}.log'; lines=log.read_text(encoding='utf-8',errors='replace').splitlines()
    crash=[i+1 for i,line in enumerate(lines) if '0xc0000374, Heap corruption' in line]
    assert crash and crash[-1]>=len(lines)-20,'No verified recent native heap error.'
    archive=root/('environment_recovery_'+a.incident)
    assert not (archive/'incident.json').exists(),'Recovery already applied.'
    target=root/'jobs'/a.failed_job/'result.json'
    assert not target.exists(),'Failure already has a result; do not overwrite it.'
    archive.mkdir(parents=True,exist_ok=True)
    for name in ['progress.json','run_control.json','active_1.json','active_2.json','supervisor.lock']:
        if (root/name).exists(): shutil.copy2(root/name,archive/name)
    row=dict(failed.get('metadata',{})); partial=target.parent/'online_progress.json'
    if partial.exists(): row.update(read(partial))
    row.update(failed['job'])
    row.update(fingerprint=failed['fingerprint'],completed=True,passed=False,status='worker_process_exit',
               reason='Verified MATLAB native heap corruption; controller exited before exact elapsed time was recorded.',
               timeoutPhase=failed['phase'],timingUnavailable=True,onlineSeconds=None,
               lastTelemetryElapsedSeconds=read(root/'progress.json')['epoch']-failed['startedEpoch'])
    write(target,row)
    other_root=(root/'jobs'/a.interrupted_job).resolve()
    resumed=not (other_root/'result.json').exists()
    if resumed:
        destination=(other_root/('interrupted_'+a.incident)).resolve()
        assert destination.is_relative_to(root.resolve())
        destination.mkdir(parents=True,exist_ok=True)
        for name in ['result.mat','online.mat','online_progress.json','first_audit.mat']:
            source=(other_root/name).resolve()
            assert source.is_relative_to(root.resolve())
            if source.exists(): shutil.move(str(source),str(destination/name))
    incident=dict(reason='Native heap corruption followed by controller restart-guard exit',
                  failedJob=a.failed_job,interruptedJob=a.interrupted_job,interruptedJobResumed=resumed,
                  recoveredUtc=dt.datetime.now(dt.timezone.utc).isoformat(),numericalImplementationUnchanged=True,
                  computeDeadlineUtc='2026-09-17T03:46:51Z')
    write(archive/'incident.json',incident)
    incidents=read(root/'runtime_incidents.json') if (root/'runtime_incidents.json').exists() else []
    incidents.append(dict(jobId=a.failed_job,kind='native_runtime_crash',reportedStatus='0xc0000374, Heap corruption',
                          sourceLog=log.name,sourceLine=crash[-1],interpretation='Native failure retained; cause not established. Exact elapsed time unavailable.'))
    write(root/'runtime_incidents.json',incidents)
    (root/'supervisor.lock').unlink(missing_ok=True)
    launched=powershell("$ProgressPreference='SilentlyContinue'; $s=New-CimInstance -ClassName Win32_ProcessStartup "
        "-ClientOnly -Property @{ShowWindow=[uint16]0;CreateFlags=[uint32]16777216}; "
        "$r=Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{"
        "CommandLine='D:\\XieZhenyu\\envs\\stewart-abfd\\Scripts\\python.exe D:\\XieZhenyu\\stewart-simscape-diffusion\\study\\launch_supervisor.py frozen_v1';"
        "CurrentDirectory='D:\\XieZhenyu\\stewart-simscape-diffusion';ProcessStartupInformation=$s}; "
        "if($r.ReturnValue -ne 0){exit 8}; $r | Select-Object ReturnValue,ProcessId | ConvertTo-Json")
    assert launched.returncode==0,launched.stderr.decode(errors='replace')
    print(launched.stdout.decode(errors='replace'))


if __name__=='__main__': main()
