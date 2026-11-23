"""Create a review snapshot under 50,000,000 bytes; never alter original evidence."""
from pathlib import Path
import datetime, hashlib, json, shutil, subprocess

root=Path(__file__).resolve().parents[2]
base=root/'results/tracking_mpc/v6'
stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
destination=root/'results/delivery'/f'stewart_tracking_mpc_v6_50hz_{stamp}'
destination.mkdir(parents=True,exist_ok=False)

def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def copy(p):
    target=destination/p.relative_to(root)
    target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(p,target)

for name in ['controller','docs','lib','mat','matlab','opt_minimal','src']:
    for p in (root/name).rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:copy(p)
copy(root/'LICENSE')

inventory=[]
for p in sorted(base.rglob('*')):
    if not p.is_file():continue
    relative=p.relative_to(base)
    # Exact run sources are useful evidence, but final source is already copied.
    include=(p.suffix.lower() in {'.json','.csv','.png','.log','.md','.txt'} and 'sources' not in relative.parts)
    include=include or p.name in {'events.mat','guarded_failure_raw.mat','compact_native_evidence.mat'}
    if p.name=='events.mat' and p.parent.name.startswith('derived_'):
        original=p.parent.parent/'events.mat'
        if original.is_file() and digest(original)==digest(p):include=False
    if p.name=='events.mat' and (p.parent/'compact_native_evidence.mat').exists():include=False
    # C-generation diagnostics stay indexed; do not distribute unaccepted binaries.
    if p.suffix.lower() in {'.c','.h','.dll','.s','.o'}:include=False
    if include:copy(p)
    inventory.append(dict(path=str(p),relative=str(p.relative_to(root)),bytes=p.stat().st_size,sha256=digest(p),included=include))

# Large native raw MATs remain indexed. The compact native bundle retains
# full motion/noise/estimate/physical signals and untouched selected event frames.
assert list(base.glob('S1_slx*/compact_native_evidence.mat')), 'Create compact actual SLX evidence before packaging'

(destination/'complete_evidence_inventory.json').write_text(json.dumps(inventory,ensure_ascii=False,indent=2),encoding='utf8')
status=dict(snapshot=stamp,fullRequestCompleted=False,formal20SeedMatrixCompleted=False,causalDeadlineClosedLoopCompleted=False,
            archiveLimitBytes=50_000_000,scope='Development repair snapshot; failures retained. See technical_progress.md and changes_r1.md.')
(destination/'delivery_status.json').write_text(json.dumps(status,ensure_ascii=False,indent=2),encoding='utf8')
(destination/'README.md').write_text('''# 50 Hz 开发修复快照

本包不是整项实验完成或实时通过声明。阅读 `docs/tracking_mpc/v6/technical_progress.md`、`changes_r1.md` 和 `model_and_clock_contract.md`。

MATLAB R2025b 中先 `addpath('controller/runs')`，再调用 `run_ch4_tracking_mpc_50hz('single', struct('runTag',"new_run"))`。重复运行必须选择新的 runTag。

原始 MAT 中未装包的大文件，在 `complete_evidence_inventory.json` 中列出本机准确路径、大小和 SHA256；没有替代或合成结果。原始路径记录用于本机证据追溯。尚未验收的实验阶段会明确报错。
''',encoding='utf8')
manifest=[dict(path=str(p.relative_to(destination)),bytes=p.stat().st_size,sha256=digest(p)) for p in sorted(destination.rglob('*')) if p.is_file()]
(destination/'package_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf8')
archive=root.parent/(destination.name+'.7z')
subprocess.run([r'C:\Program Files\7-Zip\7z.exe','a','-t7z','-mx=7','-mmt=2','-md=128m',str(archive),destination.name],cwd=destination.parent,check=True,capture_output=True)
assert archive.stat().st_size<=50_000_000, f'Archive exceeds limit: {archive.stat().st_size}; do not deliver this archive'
subprocess.run([r'C:\Program Files\7-Zip\7z.exe','t',str(archive)],check=True,capture_output=True)
result=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=digest(archive),folder=str(destination),tested=True)
(root/'results/delivery'/f'{destination.name}_receipt.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(result,ensure_ascii=False))
