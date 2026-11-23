"""Create a fresh <=50,000,000 byte portable evidence package; preserve originals."""
import datetime
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OLD = ROOT.parent/'stewart-simscape-tracking-mpc/results/delivery/stewart_tracking_mpc_20260917_203443'
V2 = ROOT/'results/tracking_mpc/v2'

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(4*1024*1024), b''):
            h.update(data)
    return h.hexdigest()

def main():
    stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    name='stewart_tracking_mpc_v2_'+stamp
    stage=ROOT/'results/delivery'/name
    stage.mkdir(parents=True, exist_ok=False)
    selected=set()
    # Reuse the already validated minimal Windows CasADi runtime/assets list.
    for old in OLD.rglob('*'):
        if old.is_file():
            rel=old.relative_to(OLD)
            if rel.parts[0] not in ('results',) and not rel.name.startswith(('PACKAGE_', 'README_')):
                if (ROOT/rel).is_file():
                    selected.add(rel)
    for folder in ('controller','docs/tracking_mpc'):
        for p in (ROOT/folder).rglob('*'):
            if p.is_file() and p.suffix.lower() in ('.m','.py','.md','.json'):
                selected.add(p.relative_to(ROOT))
    omitted=[]
    for p in V2.rglob('*'):
        if not p.is_file() or 'compact_diagnostics' in p.parts or 'delivery' in p.parts:
            continue
        rel=p.relative_to(ROOT);parts=p.relative_to(V2).parts
        include=p.suffix.lower() in ('.json','.csv','.md','.patch','.log')
        if parts[0]=='source_snapshots':
            include=True
        reason='large raw MAT retained in source project'
        if p.suffix.lower()=='.mat':
            include=parts[0] in ('tests','acceleration_replay','dense_sensor_audit','resource_audit') or p.name=='stationary_control_diagnostic.mat'
            if 'failure_packets' in parts or parts[0] in ('legacy_replay','point_repairs_final'):
                include=False;reason='compact diagnostic MAT provided under compact_diagnostics; original hash retained'
        if p.suffix.lower()=='.png':
            include='summary' in parts or ('nominal' in p.name and parts[0] in ('C_simscape','actuator_simscape'))
            reason='other genuine plots retained locally; regenerate from packaged CSV'
        if parts[0] in ('development_nominal','point_repairs'):
            include=p.suffix.lower() in ('.json','.csv') and not p.name.endswith('_constraints.csv')
            reason='superseded development detail retained locally; metrics/configuration preserved'
        if 'failure_packets' in parts and p.suffix.lower()=='.csv':
            include=False;reason='complete numerical constraints are in the adjacent diagnostic MAT'
        if include:
            selected.add(rel)
        else:
            omitted.append(dict(path=str(rel).replace('\\','/'),bytes=p.stat().st_size,sha256=sha(p),reason=reason))
    compact=V2/'compact_diagnostics'
    if compact.is_dir():
        for p in compact.rglob('*'):
            if p.is_file():
                selected.add(p.relative_to(ROOT))
    included=[]
    for rel in sorted(selected):
        src=ROOT/rel;dst=stage/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
        included.append(dict(path=str(rel).replace('\\','/'),bytes=dst.stat().st_size,sha256=sha(dst)))
    readme=ROOT/'docs/tracking_mpc/v2_delivery_readme.md'
    shutil.copy2(readme,stage/'README_v2.md')
    included.append(dict(path='README_v2.md',bytes=readme.stat().st_size,sha256=sha(readme)))
    manifest=dict(sourceRoot=str(ROOT),created=stamp,maximumArchiveBytes=50_000_000,
                  included=included,omitted=omitted,policy='immutable source files; no downsampling or fabricated results; omitted raw files remain in source project')
    (stage/'PACKAGE_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    matlab=Path('D:/MATLAB/R2025b/bin/matlab.exe')
    smoke_log=V2/'portable_smoke.log'
    matlab_stage=str(stage).replace("'", "''")
    smoke=f"restoredefaultpath; cd('{matlab_stage}'); addpath('controller/tests'); smoke_tracking_v2_portable;"
    subprocess.run([str(matlab),'-batch',smoke,'-logfile',str(smoke_log)],cwd=stage,check=True)
    assert 'PORTABLE_SMOKE_PASS' in smoke_log.read_text(encoding='utf-8',errors='replace')
    for entry in included:
        assert sha(stage/entry['path'])==entry['sha256'], 'Portable check modified packaged evidence'
    shutil.copy2(smoke_log,stage/'PORTABLE_SMOKE.log')
    manifest['included'].append(dict(path='PORTABLE_SMOKE.log',bytes=smoke_log.stat().st_size,sha256=sha(smoke_log)))
    (stage/'PACKAGE_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    archive=ROOT.parent/(name+'.7z')
    seven=Path('C:/Program Files/7-Zip/7z.exe')
    subprocess.run([str(seven),'a','-t7z','-m0=lzma2','-mx=7','-md=128m','-ms=on','-mmt=4',str(archive),name],cwd=stage.parent,check=True)
    subprocess.run([str(seven),'t',str(archive)],check=True)
    result=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=sha(archive),staging=str(stage))
    (V2/'package_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    assert archive.stat().st_size<=50_000_000, 'Archive exceeds cap; retain it and make a smaller fresh package.'
    print(json.dumps(result))

if __name__=='__main__':
    main()
