"""Local SSH helper with encoded PowerShell arguments, no shell interpolation."""
import base64
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
REMOTE='D:/XieZhenyu/stewart-simscape-diffusion'
NAME='payload_condition_expansion_v2_longtime_20260926'

def ps(script):
    sys.stdout.reconfigure(encoding='utf-8');sys.stderr.reconfigure(encoding='utf-8')
    script="$ProgressPreference='SilentlyContinue'; [Console]::OutputEncoding=[System.Text.UTF8Encoding]::new(); "+script
    encoded=base64.b64encode(script.encode('utf-16le')).decode()
    r=subprocess.run(['ssh','ws','powershell','-NoProfile','-EncodedCommand',encoded],capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=60)
    print(r.stdout);print(r.stderr,file=sys.stderr) if r.stderr else None
    if r.returncode:raise SystemExit(r.returncode)

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    action=sys.argv[1]
    if action=='push':
        ps(f"New-Item -ItemType Directory -Force -Path '{REMOTE}/analysis/payload_expansion' | Out-Null")
        for file in Path(__file__).parent.iterdir():
            if file.suffix in ['.py','.m','.json']:
                subprocess.run(['scp',str(file),f'ws:{REMOTE}/analysis/payload_expansion/{file.name}'],check=True)
    elif action=='start':
        command=f'D:/XieZhenyu/envs/stewart-abfd/Scripts/pythonw.exe {REMOTE}/analysis/payload_expansion/run_expansion.py'
        ps(f"Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{{CommandLine='{command}';CurrentDirectory='{REMOTE}'}} | ConvertTo-Json -Compress")
    elif action=='status':
        ps(f"""
$p='{REMOTE}/results/chapter3/{NAME}'
$state=Get-Content -Raw -LiteralPath ($p+'/worker_status.json') | ConvertFrom-Json
foreach($f in @('worker_status.json','finish_status.json','dataset_report.json')){{
    if(Test-Path -LiteralPath ($p+'/'+$f)){{Get-Content -LiteralPath ($p+'/'+$f)}}
}}
$a=@(Get-ChildItem -LiteralPath ($p+'/checks') -Filter expand_*.json | ForEach-Object {{Get-Content -Raw -LiteralPath $_.FullName | ConvertFrom-Json}})
[pscustomobject]@{{newCompleted=$a.Count;newAccepted=@($a | Where-Object accepted).Count;additionalAccepted=(Get-ChildItem -LiteralPath ($p+'/accepted') -Filter *.mat).Count}} | ConvertTo-Json -Compress
foreach($i in 1..8){{
    $f=$p+'/progress_worker_'+$i+'.json'
    if(Test-Path -LiteralPath $f){{$v=Get-Content -Raw -LiteralPath $f | ConvertFrom-Json; [pscustomobject]@{{worker=$i;lastIndex=$v.newConditionsAttempted;passed=$v.last.accepted;reason=$v.last.reason}} | ConvertTo-Json -Compress}}
    if($state.status -eq 'failed' -and (Test-Path -LiteralPath ($p+'/stderr_worker_'+$i+'.log'))){{Get-Content -LiteralPath ($p+'/stderr_worker_'+$i+'.log') -Tail 12}}
}}
""")
    elif action=='inspect':
        ps(f"Get-Process matlab,pythonw -ErrorAction SilentlyContinue | Select-Object Id,CPU,StartTime | ConvertTo-Json -Compress; Get-ChildItem -LiteralPath '{REMOTE}/results/chapter3/payload_formal_dataset_v1' -Filter dataset.json | Select-Object Name,Length | ConvertTo-Json -Compress")
