from pathlib import Path
import datetime,hashlib,json,shutil,subprocess
root=Path(r'D:\NPU\Matlab_code\stewart-simscape-tracking-mpc-v6-50hz')
source=root/'results/delivery/stewart_tracking_mpc_clean_50hz_20260921_180911'
name='stewart_tracking_mpc_clean_50hz_review_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
stage=root/'results/delivery'/name;stage.mkdir(parents=True,exist_ok=False)
def digest(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
omitted=[]
for p in sorted(source.rglob('*')):
 if not p.is_file():continue
 rel=p.relative_to(source)
 if p.name in {'PACKAGE_MANIFEST.json','prepare_package_mats.m','MAT_SERIALIZATION.log','PACKAGE_MAT_SERIALIZATION.json'}:continue
 if p.name=='rawRun.mat':
  omitted.append(dict(path=rel.as_posix(),bytes=(root/rel).stat().st_size,sha256=digest(root/rel),reason='Full raw MAT supplied in companion complete archive; all exported time series are included here'))
  continue
 target=stage/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
conversion=[e for e in json.loads((source/'PACKAGE_MAT_SERIALIZATION.json').read_text()) if (stage/e['path']).exists()]
(stage/'PACKAGE_MAT_SERIALIZATION.json').write_text(json.dumps(conversion,indent=2),encoding='utf8')
(stage/'OMITTED_RAW_MANIFEST.json').write_text(json.dumps(dict(completeArchive='stewart_tracking_mpc_clean_50hz_20260921_180911.7z',omitted=omitted),indent=2),encoding='utf8')
p=stage/'README.md';s=p.read_text(encoding='utf8');s=s.replace('29 组完整原始仿真结果','29 组完整时序 CSV 与摘要、N2+A1 基线/最终两组原始 MAT');s=s.replace('每组保留 rawRun.mat、配置、清单、摘要和完整时序 CSV。','每组保留配置、清单、摘要和完整时序 CSV；本精简包保留 baseline_N2_A1、final_N2_A1 两组 rawRun.mat。其余 27 组原始 MAT 完整保存在配套完整包 stewart_tracking_mpc_clean_50hz_20260921_180911.7z 中，原文件大小和哈希见 OMITTED_RAW_MANIFEST.json。');s=s.replace('本次 29 组结果全部包含。','本次 29 组表格结果全部包含，原始 MAT 的包含范围如上。Python audit_results.py 需要完整包中的 29 组原始 MAT。');p.write_text(s,encoding='utf8')
p.write_text(s.replace('29 组完整时序 CSV 与摘要、N2+A1 基线/最终两组原始 MAT','29 组完整时序 CSV 与摘要').replace('本精简包保留 baseline_N2_A1、final_N2_A1 两组 rawRun.mat。其余 27 组原始 MAT','本精简包不附原始 MAT。全部 29 组原始 MAT').replace('为满足 50,000,000 字节打包限制，仅在副本中将 rawRun.mat 改存为无内压缩 MAT，再统一使用 7z 压缩；全部变量逐一通过 MATLAB isequaln 验证，见 PACKAGE_MAT_SERIALIZATION.json。','配套完整包仅在副本中将 rawRun.mat 改存为无内压缩 MAT，再统一使用 7z 压缩；全部变量逐一通过 MATLAB isequaln 验证，见完整包中的 PACKAGE_MAT_SERIALIZATION.json。'),encoding='utf8')
manifest=[dict(path=p.relative_to(stage).as_posix(),bytes=p.stat().st_size,sha256=digest(p)) for p in sorted(stage.rglob('*')) if p.is_file()]
(stage/'PACKAGE_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
state=dict(root=str(root),stage=str(stage),name=name,archive=str(root.parent/(name+'.7z')))
Path(__file__).with_name('review_package.json').write_text(json.dumps(state,indent=2),encoding='utf8')
print(json.dumps(state),flush=True)
subprocess.run([r'C:\Program Files\7-Zip\7z.exe','a','-t7z','-m0=lzma2','-mx=5','-md=128m','-ms=on','-mmt=2',state['archive'],name],cwd=stage.parent,check=True)
print('ARCHIVE_BYTES',Path(state['archive']).stat().st_size,flush=True)
assert Path(state['archive']).stat().st_size<=50_000_000
