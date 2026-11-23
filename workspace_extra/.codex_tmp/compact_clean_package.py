exec(__import__('pathlib').Path(__file__).with_name('package_clean_research.py').read_text(encoding='utf8').split("records = []")[0].replace("name = 'stewart_tracking_mpc_clean_50hz_'", "name = 'stewart_tracking_mpc_clean_50hz_compact_'"))
previous=json.loads(Path(__file__).with_name('package_clean_research.json').read_text())
source=Path(previous['stage']);omitted=[]
for p in source.rglob('*'):
 if not p.is_file():continue
 rel=p.relative_to(source)
 if p.name=='PACKAGE_MANIFEST.json':continue
 if p.name in {'actual_constraints.csv','constraint_residuals.csv','control_diagnostics.csv'}:
  omitted.append(dict(path=rel.as_posix(),bytes=p.stat().st_size,sha256=digest(p),reason='Losslessly reproducible from included rawRun.mat using restore_campaign_csv'))
  continue
 target=stage/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
(stage/'REGENERABLE_EXPORTS.json').write_text(json.dumps(omitted,indent=2),encoding='utf8')
p=stage/'README.md';s=p.read_text(encoding='utf8');s=s.replace('每组保留 rawRun.mat、配置、清单、摘要和完整时序 CSV。','每组保留完整 rawRun.mat、配置、清单和摘要。汇总 CSV、图表已包含；87 个逐组时序 CSV 与 MAT 重复，为减小体积未直接装包，执行 `restore_campaign_csv` 可恢复并核对原文件 SHA256，详见 REGENERABLE_EXPORTS.json。');p.write_text(s,encoding='utf8')
(stage/'restore_campaign_csv.m').write_text('''function restore_campaign_csv
root=pwd;addpath(fullfile(root,'controller/tracking_mpc'));setupTrackingMpcPaths;
files=dir(fullfile(root,'results/tracking_mpc/clean_research_20260921/*/rawRun.mat'));
for k=1:numel(files)
 data=load(fullfile(files(k).folder,files(k).name));
 summary=jsondecode(fileread(fullfile(files(k).folder,'summary.json')));data.rawRun.metrics=summary.metrics;
 plant=data.ref.model;plant.scene=data.ref.scene;plant.profile=data.manifest.actualActuatorProfile;
 plant.constraintNames=[compose("length_%d",(1:6)');compose("speed_%d",(1:6)'); ...
 compose("acceleration_%d",(1:6)');"sigma_lower_squared";"collision_roof";"collision_left";"collision_right"; ...
 compose("actual_force_%d",(1:6)');compose("actual_rate_%d",(1:6)')];
 [~,caseName]=fileparts(files(k).folder);out=fullfile(root,'restored_csv',caseName);
 exportTrackingResearchRun(data.rawRun,data.ref,plant,out);
end
expected=jsondecode(fileread(fullfile(root,'REGENERABLE_EXPORTS.json')));
for k=1:numel(expected)
 [parent,name,ext]=fileparts(expected(k).path);[~,caseName]=fileparts(parent);
 p=fullfile(root,'restored_csv',caseName,[name ext]);
 assert(strcmp(trackingFileHash(p),expected(k).sha256),'CSV differs from source: %s',p);
end
fprintf('RESTORED_CSV_HASHES_PASS files=%d\\n',numel(expected));
end
''',encoding='utf8')
state=dict(root=str(root),stage=str(stage),name=name,archive=str(root.parent/(name+'.7z')))
Path(__file__).with_name('package_clean_research.json').write_text(json.dumps(state,indent=2),encoding='utf8')
manifest=[dict(path=p.relative_to(stage).as_posix(),bytes=p.stat().st_size,sha256=digest(p)) for p in sorted(stage.rglob('*')) if p.is_file()]
(stage/'PACKAGE_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
print(json.dumps(state),flush=True)
subprocess.run([r'C:\Program Files\7-Zip\7z.exe','a','-t7z','-m0=lzma2','-mx=9','-md=512m','-ms=on','-mmt=2',state['archive'],name],cwd=stage.parent,check=True)
print('ARCHIVE_BYTES',Path(state['archive']).stat().st_size,flush=True)
