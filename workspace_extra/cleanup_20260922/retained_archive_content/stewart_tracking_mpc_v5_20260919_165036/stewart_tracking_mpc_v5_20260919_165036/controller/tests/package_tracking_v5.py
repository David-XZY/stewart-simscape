"""Fresh portable package, explicit omissions, <=50 decimal MB, extraction smoke."""
from pathlib import Path
import datetime,hashlib,json,shutil,subprocess
root=Path(__file__).resolve().parents[2];results=root/'results/tracking_mpc/v5'
baseline=root.parent/'stewart-simscape-tracking-mpc-v4/results/delivery/stewart_tracking_mpc_v4_20260919_143401'
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
name='stewart_tracking_mpc_v5_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
stage=root/'results/delivery'/name;stage.mkdir(parents=True,exist_ok=False)
selected=set();omitted=[]
for license_name in ('LICENSE','NOTICE','COPYING'):
 if (root/license_name).is_file():selected.add(Path(license_name))
for e in json.loads((baseline/'PACKAGE_MANIFEST.json').read_text(encoding='utf-8-sig'))['included']:
 rel=Path(e['path'])
 if len(rel.parts)>1 and rel.parts[0]!='results' and (root/rel).is_file():selected.add(rel)
for folder in ('controller','docs/tracking_mpc'):
 for p in (root/folder).rglob('*'):
  if p.is_file() and p.suffix in ('.m','.py','.ps1','.md','.json'):selected.add(p.relative_to(root))
for p in results.rglob('*'):
 if not p.is_file() or p.name=='package_result.json':continue
 rel=p.relative_to(root);parts=p.relative_to(results).parts
 keep=p.suffix in ('.csv','.json','.md','.log','.err','.patch','.png')
 if parts[0]=='source_snapshots':keep=p.suffix=='.m'
 if p.suffix=='.mat':
  keep=parts[0] in ('fixtures','R1_recovery','R1_factorial','R2_static','R2_static_final','R4_fast','tests98','tests','regression94','physics95','parameter_maps96','compact_events','physical_violations','terminal_events','test99','runtime_matched')
 if keep:selected.add(rel)
 else:omitted.append(dict(path=rel.as_posix(),bytes=p.stat().st_size,sha256=sha(p),reason='Full raw MAT or profiler artifact retained at sourceRoot; exported series and selected complete event evidence supplied'))
for rel in sorted(selected):
 target=stage/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/rel,target)
(stage/'README_v5.md').write_text('''# Tracking MPC v5

Read docs/tracking_mpc/v5_conclusions.md and v5_implementation.md.
Windows MATLAB R2025b with the required licensed toolboxes; bundled CasADi runtime. Python report helpers require numpy/matplotlib.
MATLAB R2025b: addpath('controller/runs'); run_ch4_tracking_mpc_v5('R5_negative','_new');
Fixed original 0-7.5 s motion, Ts=.01 s, Np=20, four RK4 substeps, unchanged bounds and 120 N nominal-feedforward budget.
Archive includes runnable source/assets, exported full time series, exact terminal events, static/replay tests, source diff and SHA256.
Full original raw MAT remain at sourceRoot and are explicitly listed in PACKAGE_MANIFEST.json with size/hash.
Read limitations: offline tracking and replay timings do not establish 100 Hz operation or general MPC superiority.
''',encoding='utf8')
matlab='D:/MATLAB/R2025b/bin/matlab.exe';seven='C:/Program Files/7-Zip/7z.exe'
log=results/'portable_smoke.log'
subprocess.run([matlab,'-batch',"restoredefaultpath;addpath('controller/tests');smoke_tracking_v5_portable;",'-logfile',str(log)],cwd=stage,check=True)
assert 'PORTABLE_V5_SMOKE_PASS' in log.read_text(encoding='utf-8',errors='replace')
shutil.copy2(log,stage/'PORTABLE_SMOKE.log')
included=[dict(path=p.relative_to(stage).as_posix(),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(stage.rglob('*')) if p.is_file()]
assert len(included)==len({e['path'] for e in included})
manifest=dict(sourceRoot=str(root),baseline=str(baseline),maximumArchiveBytes=50_000_000,included=included,omitted=omitted,scope='Manifest excludes itself; full raw MAT outside archive explicitly identified, no claim of all raw data in package')
(stage/'PACKAGE_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf8')
archive=root.parent/(name+'.7z')
subprocess.run([seven,'a','-t7z','-m0=lzma2','-mx=9','-md=256m','-ms=on','-mmt=2',str(archive),name],cwd=stage.parent,check=True)
subprocess.run([seven,'t',str(archive)],check=True)
verify=root/'results/archive_verification'/name
subprocess.run([seven,'x',str(archive),'-o'+str(verify.parent),'-y'],check=True)
for e in included:
 p=verify/e['path'];assert p.stat().st_size==e['bytes'] and sha(p)==e['sha256'],e['path']
smoke=results/'extracted_smoke.log'
subprocess.run([matlab,'-batch',"restoredefaultpath;addpath('controller/tests');smoke_tracking_v5_portable;",'-logfile',str(smoke)],cwd=verify,check=True)
assert 'PORTABLE_V5_SMOKE_PASS' in smoke.read_text(encoding='utf8',errors='replace')
for e in included:
 p=verify/e['path'];assert p.stat().st_size==e['bytes'] and sha(p)==e['sha256'],e['path']
result=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=sha(archive),staging=str(stage),verifiedExtraction=str(verify),includedFiles=len(included),omittedFiles=len(omitted),uniqueManifest=True,allExtractedHashesPassed=True,extractedMatlabSmoke=True)
(results/'package_result.json').write_text(json.dumps(result,indent=2),encoding='utf8')
archive.with_suffix('.7z.sha256').write_text(result['sha256']+'  '+archive.name+'\n',encoding='utf8')
print(json.dumps(result));assert result['bytes']<=50_000_000,'Oversized attempt retained; revise packaging selection explicitly'
