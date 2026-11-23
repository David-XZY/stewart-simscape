"""Package existing diffusion evidence and exact compact trajectories under 50 MB."""
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

root=Path('D:/NPU/Matlab_code/stewart-simscape-diffusion')
work=Path(__file__).resolve().parent
target=work.parent/'扩散研究全阶段资料包_20260917.zip'
entries={}
extensions={'.json','.jsonl','.csv','.log','.err','.md','.png','.mp4','.pt'}

def add(source,name=None):
    source=Path(source)
    name=name or source.relative_to(root).as_posix()
    assert name not in entries, name
    entries[name]=source

for directory in ['diffusion','study','analysis','src','opt_minimal/core','opt_minimal/ihsid']:
    for f in sorted((root/directory).rglob('*')):
        if f.is_file() and '__pycache__' not in f.parts and f.suffix.lower() in {'.m','.py','.ps1','.md','.txt','.json'}:
            add(f)
for directory in ['results/diffusion','results/remote/diffusion','results/study']:
    for f in sorted((root/directory).rglob('*')):
        if not f.is_file() or f.suffix.lower() not in extensions: continue
        if 'exports' in f.relative_to(root).parts: continue
        if f.relative_to(root).as_posix()=='results/remote/diffusion/pilot_v4/training/checkpoint.pt': continue
        add(f)
for relative in ['LICENSE','opt_minimal/README.md',
                 'results/remote/diffusion/pilot_v4/manifest.mat',
                 'results/remote/diffusion/formal_v1/manifest.mat',
                 'results/study/frozen_v1/manifest.mat',
                 'results/study/frozen_v1/jobs.mat']:
    add(root/relative)
for name in ['先读我.md','背景与实验脉络.md']: add(work/name,name)
for name in ['compact_trajectories.mat','compact_trajectory_index.json']:
    add(work/name,'compact_data/'+name)
add(work/'exportCompactDiffusion.m','compact_data/exportCompactDiffusion.m')
add(work/'export.log','compact_data/export_verification.log')

index=json.loads((work/'compact_trajectory_index.json').read_text(encoding='utf-8-sig'))
assert index['metadata']['records']==1766
manifest=dict(schemaVersion=1,sizeLimitBytes=50_000_000,
    sourceCommit=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip(),
    compactMetadata=index['metadata'],
    included='All available diffusion phase metadata, result tables, scalar audits, training assets, figures, animations, source, and compact candidate/trajectory representations.',
    omitted='Bulk original dense MAT, per-point exports JSON, old ZIP snapshots, third-party binaries and unrelated control/Simscape history. Pilot optimizer checkpoint.pt is omitted; its complete MATLAB-loadable model.json weights remain. Formal checkpoint.pt is retained.',
    files=[dict(path=name,bytes=f.stat().st_size,sha256=hashlib.sha256(f.read_bytes()).hexdigest(),source=str(f)) for name,f in sorted(entries.items())])
with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as archive:
    for name,f in sorted(entries.items()): archive.write(f,name)
    archive.writestr('文件清单.json',json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
assert target.stat().st_size<50_000_000,f'Package too large: {target.stat().st_size}'
with zipfile.ZipFile(target) as archive:
    assert archive.testzip() is None
    for item in manifest['files']:
        assert hashlib.sha256(archive.read(item['path'])).hexdigest()==item['sha256'],item['path']
verification=dict(file=str(target),bytes=target.stat().st_size,decimalMB=target.stat().st_size/1e6,
    MiB=target.stat().st_size/2**20,files=len(entries)+1,zipCRC='passed',allFileHashes='passed',
    compactRoundtrip='passed',compactRecords=index['metadata']['records'],
    sha256=hashlib.sha256(target.read_bytes()).hexdigest())
(work/'package_verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(verification,ensure_ascii=False,indent=2))
