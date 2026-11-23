from pathlib import Path
import datetime, hashlib, json, shutil, subprocess

root = Path(r'D:\NPU\Matlab_code\stewart-simscape-tracking-mpc-v6-50hz')
name = 'stewart_tracking_mpc_clean_50hz_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
stage = root / 'results/delivery' / name
stage.mkdir(parents=True, exist_ok=False)
campaign = Path('results/tracking_mpc/clean_research_20260921')
def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda: f.read(4*1024*1024), b''): h.update(chunk)
    return h.hexdigest()
records = []
for folder in ['controller','docs','lib','mat','matlab','opt_minimal','src',str(campaign)]:
    for p in sorted((root/folder).rglob('*')):
        if not p.is_file() or any(x in p.parts for x in ['python_deps','__pycache__']): continue
        rel = p.relative_to(root)
        target = stage/rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, target)
        records.append(dict(path=rel.as_posix(), bytes=p.stat().st_size, sha256=digest(p)))
shutil.copy2(root/'LICENSE',stage/'LICENSE')
records.append(dict(path='LICENSE',bytes=(root/'LICENSE').stat().st_size,sha256=digest(root/'LICENSE')))
(stage/'SOURCE_COPY_MANIFEST.json').write_text(json.dumps(records,indent=2),encoding='utf8')
(stage/'README.md').write_text('''# Stewart 50 Hz MPC：结构简化、修复与参数实验（2026-09-21）

本包包括当前工程代码、Windows CasADi 运行依赖、原始参考/模型、29 组完整原始仿真结果、报告和图表。
先看 `results/tracking_mpc/clean_research_20260921/report.md`，对比图为同目录 `baseline_vs_final.png`。
29 组均为 7.5 s rigid RK4 仿真，不代表原生 Simscape 或 50 Hz 实时验收。N2 工况仍有碰撞约束违反；失败与违反记录完整保留。
`docs` 下其他版本说明与旧打包脚本是历史资料，本次结论以上述 report.md 为准。

## 复现

Windows MATLAB R2025b，需 Optimization Toolbox、Control System Toolbox；运行 SLX 另需 Simulink、Simscape、Simscape Multibody。CasADi 已随包提供。
将 MATLAB 当前目录切换到解压后的本目录，执行：

```matlab
addpath('controller/runs');
run_ch4_tracking_mpc_final('N2','A1');
```

最终配置：MPC 20 ms、传感/估计 10 ms、Np=30、rateWeight=0.1、保守 EKF、扰动注入关闭。输出使用新的 runTag。
重跑整套实验（选择尚不存在的输出目录）：

```matlab
run_ch4_tracking_mpc_50hz('research',struct('outputFolder','results/tracking_mpc/new_clean_campaign'));
```

关键契约测试：

```matlab
addpath('controller/tests');
test_112_tracking_research('results/tracking_mpc/new_clean_tests');
```

Python 图表/审计脚本另需 numpy、scipy、matplotlib（未附带临时 Python 环境）。MATLAB 仿真不依赖 Python。

## 数据与校验

每组保留 rawRun.mat、配置、清单、摘要和完整时序 CSV。为满足 50,000,000 字节打包限制，仅在副本中将 rawRun.mat 改存为无内压缩 MAT，再统一使用 7z 压缩；全部变量逐一通过 MATLAB isequaln 验证，见 PACKAGE_MAT_SERIALIZATION.json。原工程和原始结果均未修改。
SOURCE_COPY_MANIFEST.json 记录原文件哈希；PACKAGE_MANIFEST.json 记录包内文件哈希（不包含清单自身）。MAT 内记录的旧绝对路径仅用于历史来源追溯，运行入口按解压目录解析依赖。
旧版本结果、构建缓存和临时 Python 环境不在本次包内；本次 29 组结果全部包含。
''',encoding='utf8')
(stage/'prepare_package_mats.m').write_text('''function prepare_package_mats
root=pwd; addpath(fullfile(root,'controller/tracking_mpc'));
files=dir(fullfile(root,'results/tracking_mpc/clean_research_20260921/*/rawRun.mat'));
assert(numel(files)==29); records=cell(1,numel(files));
for k=1:numel(files)
 p=fullfile(files(k).folder,files(k).name); before=trackingFileHash(p);
 data=load(p); save(p,'-struct','data','-v7','-nocompression'); restored=load(p);
 assert(isequaln(data,restored),'Container conversion changed variables');
 records{k}=struct('path',erase(string(p),string(root)+filesep),'sourceSHA256',before, ...
 'packagedSHA256',trackingFileHash(p),'allVariablesExactlyEqual',true);
 fprintf('MAT_EQUIVALENCE %d/29 %s\\n',k,files(k).folder);
end
fid=fopen(fullfile(root,'PACKAGE_MAT_SERIALIZATION.json'),'w');
fprintf(fid,'%s',jsonencode([records{:}],PrettyPrint=true)); fclose(fid);
fprintf('MAT_CONTAINER_EQUIVALENCE_PASS files=29\\n');
end
''',encoding='utf8')
state = dict(root=str(root),stage=str(stage),name=name,archive=str(root.parent/(name+'.7z')))
Path(__file__).with_suffix('.json').write_text(json.dumps(state,indent=2),encoding='utf8')
print(json.dumps(state),flush=True)
subprocess.run([r'D:\MATLAB\R2025b\bin\matlab.exe','-batch','restoredefaultpath;prepare_package_mats;','-logfile',str(stage/'MAT_SERIALIZATION.log')],cwd=stage,check=True)
manifest=[dict(path=p.relative_to(stage).as_posix(),bytes=p.stat().st_size,sha256=digest(p)) for p in sorted(stage.rglob('*')) if p.is_file()]
(stage/'PACKAGE_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
subprocess.run([r'C:\Program Files\7-Zip\7z.exe','a','-t7z','-m0=lzma2','-mx=9','-md=256m','-ms=on','-mmt=2',state['archive'],name],cwd=stage.parent,check=True)
print('ARCHIVE_BYTES',Path(state['archive']).stat().st_size,flush=True)
