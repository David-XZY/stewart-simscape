"""Local deployment, detached start, compact status, and result collection."""
import argparse
import base64
import json
from pathlib import Path
import subprocess
from common import HERE,ROOT,NAME

REMOTE='D:/XieZhenyu/stewart-simscape-diffusion'
PYTHON='D:/XieZhenyu/envs/stewart-abfd/Scripts/pythonw.exe'

def ps(script):
    script="[Console]::OutputEncoding=[System.Text.UTF8Encoding]::new(); "+script
    encoded=base64.b64encode(script.encode('utf-16le')).decode()
    r=subprocess.run(['ssh','-o','ConnectTimeout=15','ws','powershell','-NoProfile','-EncodedCommand',encoded],capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=60)
    print(r.stdout)
    if r.returncode:raise RuntimeError(r.stderr)

def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['deploy','start','status','collect']);args=p.parse_args()
    if args.action=='deploy':
        ps(f"New-Item -ItemType Directory -Force -Path '{REMOTE}/analysis/payload_retrain474' | Out-Null")
        for file in HERE.iterdir():
            if file.suffix in ['.py','.m','.json']:subprocess.run(['scp',str(file),f'ws:{REMOTE}/analysis/payload_retrain474/{file.name}'],check=True)
    elif args.action=='start':
        ps(f"Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{{CommandLine='{PYTHON} {REMOTE}/analysis/payload_retrain474/runner.py --worker';CurrentDirectory='{REMOTE}'}} | ConvertTo-Json -Compress")
    elif args.action=='status':
        ps(f"""
$root='{REMOTE}/results/chapter3/{NAME}'
Get-Content -LiteralPath ($root+'/status.json') -ErrorAction SilentlyContinue
Get-ChildItem -LiteralPath ($root+'/models') -Directory -ErrorAction SilentlyContinue | ForEach-Object {{
 $p=Join-Path $_.FullName 'progress.json'; if(Test-Path -LiteralPath $p){{$v=Get-Content -Raw -LiteralPath $p | ConvertFrom-Json;$last=$v.log[-1];[pscustomobject]@{{model=$_.Name;updates=$v.updates;epoch=$last.epoch;bestEpoch=$v.bestEpoch;physicalRate=$last.physicalRate;seconds=$v.elapsedSeconds}}|ConvertTo-Json -Compress}}
}}
Get-Content -LiteralPath ($root+'/benchmark_progress.json') -ErrorAction SilentlyContinue
""")
    else:
        # Explicitly fetch this experiment only; no test artifacts or source deletions.
        target=ROOT/'results/chapter3'/NAME;target.mkdir(parents=True,exist_ok=True)
        subprocess.run(['scp','-r',f'ws:{REMOTE}/results/chapter3/{NAME}/.',str(target)],check=True)

if __name__=='__main__':main()
