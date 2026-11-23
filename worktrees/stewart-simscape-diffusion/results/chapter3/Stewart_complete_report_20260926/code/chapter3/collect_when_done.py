"""Collect the completed experiment automatically; do not alter numerical records."""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import zipfile


def remote(script, timeout=45):
    script="$ProgressPreference='SilentlyContinue'; $ErrorActionPreference='Stop'; "+script
    encoded=base64.b64encode(script.encode('utf-16-le')).decode('ascii')
    result=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','ws','powershell.exe',
        '-NoProfile','-NonInteractive','-OutputFormat','Text','-EncodedCommand',encoded],
        capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=timeout,
        creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    if result.returncode: raise RuntimeError(result.stderr[-1000:])
    return result.stdout.strip()


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--run',required=True); args=parser.parse_args()
    assert all(c.isalnum() or c in '_-' for c in args.run)
    root=Path(__file__).resolve().parents[1]; parent=root/'results/chapter3'; target=parent/args.run; target.mkdir(parents=True,exist_ok=True)
    lock=target/'collector.lock'; fd=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY); os.write(fd,str(os.getpid()).encode()); os.close(fd)
    remote_run=f'D:/XieZhenyu/stewart-simscape-diffusion/results/chapter3/{args.run}'
    def status(**values):
        tmp=target/'collection_status.tmp'; tmp.write_text(json.dumps(dict(updatedEpoch=time.time(),**values),ensure_ascii=False,indent=2),encoding='utf-8'); tmp.replace(target/'collection_status.json')
    try:
        while True:
            try:
                response=remote(f"""
                $folder='{remote_run}'
                $ready=Test-Path ($folder+'/delivery_ready.json')
                $stop=Test-Path ($folder+'/safety_stop.json')
                $running=$false
                if(Test-Path ($folder+'/controller.lock')) {{ $workerId=[int](Get-Content ($folder+'/controller.lock')); $running=$null -ne (Get-Process -Id $workerId -ErrorAction SilentlyContinue) }}
                $progress=$null
                if(Test-Path ($folder+'/progress.json')) {{$progress=Get-Content ($folder+'/progress.json') -Raw | ConvertFrom-Json}}
                @{{ready=$ready;stop=$stop;running=$running;progress=$progress}} | ConvertTo-Json -Depth 5 -Compress
                """)
                observed=json.loads(response); status(state='waiting',remote=observed)
                if observed['ready']: break
                if observed['stop'] or not observed['running']:
                    status(state='needs_attention',remote=observed); return
            except (OSError,RuntimeError,ValueError,subprocess.TimeoutExpired) as error:
                status(state='connection_retry',error=str(error))
            time.sleep(60)
        status(state='archiving')
        archive=remote_run+'_delivery.zip'
        expected=remote(f"Compress-Archive -LiteralPath '{remote_run}' -DestinationPath '{archive}' -Force; (Get-FileHash -LiteralPath '{archive}' -Algorithm SHA256).Hash",timeout=3600).splitlines()[-1].lower()
        local=parent/(args.run+'_delivery.zip'); status(state='downloading')
        subprocess.run(['scp',f'ws:{archive}',str(local)],check=True,timeout=7200,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        actual=hashlib.file_digest(local.open('rb'),'sha256').hexdigest(); assert actual==expected,'Archive hash mismatch'
        status(state='extracting')
        with zipfile.ZipFile(local) as package:
            for entry in package.infolist():
                name=Path(entry.filename); resolved=(parent/name).resolve()
                assert name.parts and name.parts[0]==args.run and resolved.is_relative_to(target.resolve()),'Archive path outside run'
            package.extractall(parent)
        summary=json.loads((target/'report/summary.json').read_text(encoding='utf-8'))
        assert summary['complete'] and not summary['integrityErrors']
        status(state='complete',archiveSha256=actual,summary=summary,materials=str(target/'report/chapter3_materials.md'))
    finally:
        lock.unlink(missing_ok=True)


if __name__=='__main__': main()
