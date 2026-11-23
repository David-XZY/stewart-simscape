"""Create reviewable route/path audit artifacts from the read-only inventory."""
from pathlib import Path
import collections
import datetime as dt
import hashlib
import json
import zipfile

OUT=Path(__file__).resolve().parent
ROOT=Path(r'D:\NPU\Matlab_code');U=ROOT/'stewart_unified_nmpc'

def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def write(p,v):Path(p).write_text(json.dumps(v,ensure_ascii=False,indent=2),encoding='utf-8')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def link(p,label=None):
    p=Path(p)
    return f'[{label or p.name}](<{p.as_posix()}>)'

# These are source and location descriptions, not executable instructions.
GROUPS=[
('当前总入口与公共物理',U,[
 ('DEVELOPMENT_CH4_NOOPT_STATUS.json','第四章当前状态；优先于旧根 README'),
 ('DEVELOPMENT_CH5_ADAPTIVE_STATUS.json','第五章当前状态；实时仍未通过'),
 ('docs/source_design/long_payload_proposal/proposed_parameters.json','长载荷几何、惯性、物理限值来源；文件提案状态字段属于历史来源，当前使用由运行快照确认'),
 ('+sn/config.m','公共配置及物理量读取'),('+sn/kernel.m','统一运动学、动力学、逆动力学与约束内核'),
 ('+sn/repair_kernel.m','机构模型修订内核'),
 ('taskset_v3','60 任务生成、几何、读取和核验代码'),
 ('tasksets/core_v3_20261111/manifest.json','60 任务及冻结文件哈希'),
 ('tasksets/core_v3_20261111/tasks','逐任务场景、端点、初值和验证证据'),
 ('tasksets/core_v3_20261111/index.html','60 个场景的可视图库'),
 ('models/long_payload_v2_20261109','长载荷基础 Simulink/Simscape 模型'),
 ('models/ch4_noopt_20261121','当前无在线优化跟踪模型'),
]),
('第三章 IHSID 与第四章所用参考',U,[
 ('plan_core_ihsid.m','第三章定义驱动的独立 IHSID 入口；不等于第四章最终带执行余量的参考入口'),
 ('+sn/ch3_config.m','当前独立 HS2 分支配置；默认 timeWeight=0.2'),
 ('+sn/ch3_problem.m','自由时长、HS2 运动学、逆动力学、原代价'),
 ('+sn/ch3_shape_cost.m','腿/载荷/角加速度和驱动力变化代价'),
 ('+sn/ch3_hs2_consistency.m','导出曲线与二阶配点一致性检查'),
 ('ch3_ihsid','完整规划、独立审计、自由时间/HS2 对照、绘图与报告'),
 ('ch4_noopt/plan_reference_smooth_v6.m','带连续包络的原目标离线求解'),
 ('ch4_noopt/plan_reference_penalty_v4.m','同一原目标的弹性恢复与求解状态延续'),
 ('ch4_noopt/plan_reference_adaptive.m','按证明失败单元细分，随后零松弛性能优化'),
 ('ch4_noopt/+c4n/reference_template_smooth.m','连续 Bernstein / 分离平面优化表达'),
 ('ch4_noopt/+c4n/refresh_plane_witnesses.m','离线凸包最近点选择证明方向'),
 ('ch4_noopt/+c4n/adaptive_reference_cells.m','保留 HS2 曲线变量的包络单元细分'),
 ('results/ch4_noopt_20261121/references/uniform_bernstein_v1/registry.json','当前 60 条正式参考的唯一选择表；实际 MAT 分散在不同尝试目录'),
 ('results/ch4_noopt_20261121/references','当前参考、来源、失败和重试历史'),
 ('results/ch3_ihsid_20261117','更早固定时长结果'),
 ('results/ch3_ihsid_free_time_20261118','自由时长早期结果与原理诊断'),
 ('results/ch3_ihsid_hs2_20261118','HS2 一致性与可选软项对照'),
 ('results/ch3_ihsid_slow_20261120','时间权重研究结果'),
]),
('第四章当前无在线优化跟踪',U,[
 ('ch4_noopt/METHOD.md','SOCP 必要性、观测器、参考余量和证明边界'),
 ('ch4_noopt/run_ch4_noopt_selected.m','选定统一配置复现入口；显式传入辨识文件'),
 ('ch4_noopt/run_ch4_noopt.m','独立 Simscape 闭环入口'),
 ('ch4_noopt/calibration.py','仅从独立测量数据辨识质量与惯性'),
 ('ch4_noopt/+c4n/apply_calibration.m','将辨识值及误差区间装入控制模型'),
 ('ch4_noopt/+c4n/design.m','18 维移动参考仿射 TVLQI，反向 Riccati 递推'),
 ('ch4_noopt/+c4n/kernel.m','控制/观测器表达与离散线性化'),
 ('ch4_noopt/+c4n/step.m','直接合成实际腿力；无在线 QP/SOCP'),
 ('ch4_noopt/control_gateway.c','原生快速控制接口'),('ch4_noopt/rollout_native.c','完整机构植物及控制闭环原生执行器'),
 ('ch4_noopt/window_tube.py','含全历史的条件增广线性误差传播'),
 ('ch4_noopt/revalidate_reference_margin.m','新参考下重新计算余量并复核固定曲线'),
 ('ch4_noopt/stream_validation.py','统一批量设计与执行'),('ch4_noopt/refine_integration.py','积分步长减半复验'),
 ('results/ch4_noopt_20261121/calibration/v3','独立辨识及测量来源'),
 ('results/ch4_noopt_20261121/designs/new_references_v1','360 份参考/辨识模型设计及本地控制器库'),
 ('results/ch4_noopt_20261121/reference_margin_new_v1.json','选定统一周期和执行裕度'),
 ('results/ch4_noopt_20261121/reference_margin_revalidation_new_v1.json','固定参考在重算裕度下的复核'),
 ('results/ch4_noopt_20261121/native_campaigns/new_references_v1','480 组主闭环记录'),
 ('results/ch4_noopt_20261121/integration_refinement','480 组积分减半记录'),
 ('results/ch4_noopt_20261121/counterfactuals','12 代表任务、参考与周期的 2×2 对照'),
 ('results/ch4_noopt_20261121/crosschecks','Simscape 交叉核对'),
 ('results/ch4_noopt_20261121/delivery_v1/REPORT.md','当前第四章正式数值汇总'),
 ('results/ch4_noopt_20261121/delivery_v1/final_verification.json','第四章交付验收回执'),
]),
('第五章当前 NMPC 与异步 IHSID',U,[
 ('ch5_adaptive/METHOD.md','完整方法、对照及适用条件'),('ch5_adaptive/policy.py','统一周期、到达条件、物理限制和保护清单'),
 ('ch5_adaptive/run_case.py','双频闭环主入口与监督器'),('ch5_adaptive/events.py','事件通知；规划信息与植物真实扰动分离'),
 ('ch5_adaptive/model.py','原参考求值、精确逆动力学、候选反馈设计'),('ch5_adaptive/nmpc.py','IPOPT 冷启动及原曲线软引导'),
 ('ch5_adaptive/rti.py','Gauss–Newton SQP-RTI 与重新滚动验收'),('ch5_adaptive/hpipm.py','固定版本结构化 QP 求解器接口'),
 ('ch5_adaptive/stages.py','统一原物理约束、目标和平滑成本、执行余量收紧'),
 ('ch5_adaptive/progress.py','剩余弧长上界与终点物理位置耦合'),('ch5_adaptive/bounds.py','候选专属条件线性误差及力预算 B'),
 ('ch5_adaptive/packets.py','稠密动态参考、控制增益、发布检查与通用停止候选'),
 ('ch5_adaptive/completion.py','同一场景下后续更新不得推迟已承诺到达时刻'),
 ('ch5_adaptive/runtime.py','持续执行；读取原第四章控制器/植物库'),('ch5_adaptive/runtime.c','保存积分、观测器和真实上一拍力历史'),
 ('ch5_adaptive/audit.py','实际状态、实际加速度、力和整机几何的独立验收'),
 ('ch5_adaptive/replanner.py','目标/场景版本化、隐藏 MATLAB 后台 IHSID、迟到结果管理'),
 ('ch5_adaptive/ch5_ihsid_worker.m','后台全局 IHSID 入口'),('ch5_adaptive/ch5_reference_seed.m','统一几何尺度的 50 初值集'),
 ('ch5_adaptive/ch5_project_kinematics.m','HS2 仿射运动学精确投影'),('ch5_adaptive/ch5_publish_incumbent.m','原问题零松弛可行性独立验收后提前发布'),
 ('ch5_adaptive/regression.py','原 480 组重新执行'),('ch5_adaptive/stream_regression.py','持续执行器与旧执行记录等价性'),
 ('ch5_adaptive/campaign.py','批处理与参数记录'),('ch5_adaptive/simscape.py','独立植物执行时序复核'),
 ('ch5_adaptive/report.py','汇总全部正面与负面证据'),
 ('results/ch5_adaptive_20261123/delivery_v1/REPORT.md','当前第五章正式汇总'),
 ('results/ch5_adaptive_20261123/delivery_v1/final_verification.json','机器可读逐项证据与计时'),
 ('results/ch5_adaptive_20261123/campaigns','60 名义接入、动态对照、积分减半和延时注入'),
 ('results/ch5_adaptive_20261123/cases','完整持续挡路、同备用机制对照等原始记录'),
 ('results/ch5_adaptive_20261123/diagnostics','完整决策计时、机制与实现诊断'),
 ('results/ch5_adaptive_20261123/source_snapshots','各次运行对应的源码快照'),
 ('results/ch5_adaptive_20261123/simscape','两项 Simscape 执行时序复核'),
]),
('旧方案、共用资产与旧说明',U,[
 ('README.md','停留在 11 月 16 日旧 NMPC 阶段，不能作为当前总状态'),
 ('ch4_control','上一版 SOCP/TVLQI；公共配置/审计仍有复用'),('ch4_reporting','旧第四章报告代码'),
 ('results/ch4_tracking_20261120','旧 SOCP 对照、原参考与原脉冲时间来源'),
 ('results/ch4_tracking_20261120/references.json','当前代码仍引用的旧参考来源表'),
 ('core_integration','公共时标接入阶段'),('core_repair','动力学/闭环一致性修订阶段'),
 ('nmpc_rebuild','旧 NMPC 重构分支'),('nmpc_simple','旧简化控制分支'),('nmpc_rti','旧实时迭代分支；当前仍保留保护与依赖'),('nmpc_guidance','旧几何引导分支'),
 ('train_diffusion.m','旧统一扩散训练入口；当前第五章未启用'),('infer_diffusion.m','旧统一扩散推理入口'),('evaluate_diffusion.m','旧扩散验收入口'),
 ('history','统一工程各次修改前的源码历史'),('deliverables','统一工程旧审阅包、模型包和可读导出'),
 ('docs','数学、模型来源、阶段设计文档；必须按日期辨别'),
]),
('运行依赖，不能按 cache 名字当作无用内容',U,[
 ('cache/long_payload_v2_20261109/python/Scripts/python.exe','本轮及当前工程使用的项目 Python'),
 ('cache/ch5_adaptive_20261123','当前第五章导出图、原生库和编译记录'),
 ('cache/ch5_adaptive_20261123/python_packages','当前第五章独立 Python 包目录'),
 ('cache/nmpc_rti_20261115/vendor_simd','当前仍调用的 HPIPM/BLASFEO 固定库'),
 ('third_party/casadi','MATLAB CasADi'),
 ('results/ch4_noopt_20261121/full_plant_v2','持续执行器读取的完整机构植物库'),
]),
('Matlab_code 内其他保留工程和产物',ROOT,[
 ('stewart-simscape','最早 Simscape 主工程；其 .git 是关联工作树共享对象库'),
 ('stewart-simscape/.git','Git 已提交历史；不等于所有已删除未跟踪结果的备份'),
 ('stewart-simscape/controller','早期 PID、计算力矩 LQI、QP 代码'),
 ('stewart-simscape/opt_minimal','早期 IHSID / 轨迹优化源码与基准'),
 ('stewart-simscape-disturbance-experiment','早期扰动控制对照工作树'),
 ('stewart-simscape-diffusion','旧扩散+IHSID 独立工作树'),
 ('stewart-simscape-diffusion/diffusion','旧候选生成、训练、物理引导与 IHSID 精修代码'),
 ('stewart-simscape-diffusion/results/remote/diffusion','已拉回本机的早期扩散实验记录；是否完整依逐运行证据'),
 ('stewart-simscape-ch5-integration/chapter5','旧扩散控制增量与控制集成工程'),
 ('stewart-simscape-ch5-integration/chapter5/artifacts','旧第五章任务、教师、训练/对照与结果'),
 ('output','早期核心代码包、毕业设计归档、公式、Visio/PPT 图等；不等于已删除 deliveries'),
 ('output/Stewart_毕业设计归档_20260828.zip','仍保留的 8 月历史包'),
 ('output/Stewart_全流程核心代码_20260904.zip','仍保留的 9 月核心代码包'),
 ('output/扩散研究全阶段资料包_20260917.zip','仍保留的 9 月扩散资料包'),
 ('outputs','早期在线规划代码/论文、流程图和本次索引'),
 ('outputs/online_planning_control_20261014','当前仅剩 native/papers/preflight 等；不把曾经存在的旧实验文件说成仍完整保留'),
 ('deliverables','文档转换及答辩录音整理产物，复数 deliveries 已删除'),
 ('references','根目录参考资料'),('.thesis_control_work','早期论文/控制文字生成工作文件'),
 ('cleanup_20260922','9 月历史清理记录'),('cleanup_20261117','11 月归档清单、哈希及清理记录；原 archive 已在 11 月 23 日删除'),
 ('cleanup_20261123/delete_groups_3_4_10','旧 MPC / ABFD / 归档删除操作、原路径清单和核验'),
]),
('项目之外的论文、原型和资料','D:/NPU',[
 ('毕业设计/论文/main.typ','现存 Typst 总入口'),('毕业设计/论文/chapters','第三至第五章文字；主要仍为旧方案'),
 ('毕业设计/论文/论文.pdf','现存论文 PDF，不等于本轮新算法报告'),('毕业设计/论文/figures','论文插图'),('毕业设计/论文/references.bib','论文参考文献'),
 ('毕业设计/本人编写','用户手写/主写论文与材料'),('毕业设计/AI编写','历次辅助生成的论文/材料'),
 ('毕业设计/ABFD_现有代码与结果_20260914.zip','ABFD 旧工作树删除后仍存在的 9 月快照；不是最后状态完整备份'),
 ('毕业设计/nmpc_complete_design_sources.zip','NMPC 设计来源压缩包'),
 ('并联机器人','开题、中期、建模、文献、硬件采购/选型/测试、旧 MATLAB 与 STM32 资料'),
 ('Matlab_workspace','更早 MATLAB Project，含 src/scripts/simulink/Stewart6UCU'),
 ('Matlab_StewartPlatform','早期 .mdl 参考模型与控制器'),('matlab_example/m2m_StewartPlatform','示例工程中的 Stewart 子目录'),
 ('Adams_Workspace','Stewart 的 ADAMS 建模与联仿文件'),
 ('STM_Project/SGP_Control_System','STM32 执行器控制工程：位置/速度串级 PID、编码器、PWM、串口；未证明与当前新算法完成实机集成'),
 ('STM_Project/Linear_Actuator_Control_test','电缸控制早期测试'),('STM_Project','其余开发板/执行器基础测试工程'),
 ('并行实时控制','多核/实时控制方案与文献'),('组会PPT','组会汇报与架构图'),('周汇报','周汇报'),('专业实践','实践材料'),
 ('简历及相关资料/Stewart毕业设计技术面试问答.md','项目面试问答'),('robot','相关机器人材料；不一概归入当前 Stewart 主线'),
]),
('下载与 C 盘外部工作树','D:/Downloads',[
 ('Stewart 并联机器人第三至第五章研究路线与扩散切换方案.docx','本轮核对的原方案底稿；不是完成声明'),
 ('Stewart_路径规划与控制方法目录.xlsx','方法目录'),('stewart_long_payload_proposal_20261109.zip','长载荷参数提案源包'),
 ('stewart_smooth_nmpc_revision_20261110.zip','早期平顺 NMPC 修订提案包'),
 ('nmpc_formulation.md','NMPC 公式文档，同目录还有 typ/tex/pdf'),('IDM/Documents','下载的论文资料；文件名发现清单列出匹配项'),
 ('IDM/Compressed/stewart-simscape-master.zip','原参考工程包'),
]),
('外部工作树与实际程序依赖','C:/',[
 ('Users/David_X/.config/superpowers/worktrees/stewart-simscape/sc-mpc-control','仍登记在 Git 的早期 MPC 工作树，位于 Matlab_code 外'),
 ('Users/David_X/Desktop/results','早期仿真视频及结果'),('msys64/ucrt64/bin/gcc.exe','原生 C 与 CasADi JIT 编译器'),
]),
('MATLAB 安装位置','D:/',[('MATLAB/R2025b/bin/matlab.exe','当前代码中使用的 MATLAB 可执行文件；工具箱许可未在本轮重新运行核验')]),
]

def make():
    scope=read(OUT/'scope.json');folders=read(OUT/'directory_summary.json')
    now=dt.datetime.now().astimezone().isoformat()
    checked=[]
    for group,base,items in GROUPS:
        for relative,role in items:
            p=Path(base)/relative
            checked.append(dict(group=group,path=str(p),role=role,exists=p.exists(),kind='directory' if p.is_dir() else 'file' if p.is_file() else 'absent'))
    write(OUT/'key_paths.json',checked)
    baseline=read(U/'results/ch5_adaptive_20261123/baseline.json')['files']
    changed=[name for name,expected in baseline.items() if not (U/name).is_file() or sha(U/name)!=expected]
    registry=read(U/'results/ch4_noopt_20261121/references/uniform_bernstein_v1/registry.json')
    references=[]
    for row in registry['entries']:
        selected=row['selected'];p=Path(selected['file']);ok=p.is_file() and sha(p)==selected['sha256']
        references.append(dict(task=row['task_id'],path=str(p),duration_s=selected['duration_s'],sha256=selected['sha256'],hash_matches=ok))
    write(OUT/'current_60_reference_paths.json',references)
    design_records=list((U/'results/ch4_noopt_20261121/designs/new_references_v1').glob('*/*/design_record.json'))
    designs=[]
    for receipt in design_records:
        r=read(receipt);mat=receipt.parent/'design.mat';native=Path(r['native_folder'])/'controller.dll'
        designs.append(dict(folder=str(receipt.parent),design_exists=mat.is_file(),controller=str(native),controller_exists=native.is_file()))
    write(OUT/'current_360_design_paths.json',designs)
    evid=[U/'DEVELOPMENT_CH4_NOOPT_STATUS.json',U/'DEVELOPMENT_CH5_ADAPTIVE_STATUS.json',U/'ch4_noopt/METHOD.md',U/'ch5_adaptive/METHOD.md',U/'ch5_adaptive/model.py',U/'ch5_adaptive/nmpc.py',U/'results/ch4_noopt_20261121/delivery_v1/REPORT.md',U/'results/ch5_adaptive_20261123/delivery_v1/REPORT.md',U/'results/ch5_adaptive_20261123/delivery_v1/final_verification.json',ROOT/'cleanup_20261123/delete_groups_3_4_10/verification.json',Path(r'D:\NPU\毕业设计\论文\chapters\03-轨迹规划.typ'),Path(r'D:\NPU\毕业设计\论文\chapters\04-轨迹跟踪控制.typ'),Path(r'D:\NPU\毕业设计\论文\chapters\05-规划控制一体化.typ')]
    write(OUT/'evidence_hashes.json',[dict(path=str(p),sha256=sha(p)) for p in evid])
    verification=dict(created=now,inventory_files=sum(t['files'] for t in scope['totals'].values()),fully_scanned_roots=len(scope['full_roots']),scan_errors=scope['errors'],skipped_reparse_points=len(scope['reparse_points']),checked_key_paths=len(checked),missing_key_paths=[v for v in checked if not v['exists']],protected_files=len(baseline),protected_changes=changed,references=len(references),reference_hash_failures=[x['task'] for x in references if not x['hash_matches']],designs=len(designs),design_path_failures=[x for x in designs if not x['design_exists'] or not x['controller_exists']],scope='Location/source/hash audit; no new controller experiment or hardware test.')
    write(OUT/'audit_verification.json',verification)
    lines=['**Stewart 全项目路径索引｜2026-11-23 本机核对**','',f"完整扫描 {len(scope['full_roots'])} 个目录树，加上下载、桌面、Documents 和 D:/NPU 的相关文件名检索，共记录 {verification['inventory_files']:,} 个文件。完整原始清单为 {link(OUT/'files.jsonl')}；它含依赖、生成文件与历史结果，并不表示同样数量的算法源码。",'',f"扫描错误 {len(scope['errors'])}；跳过 {len(scope['reparse_points'])} 个目录联接且记录其目标。未扫描远端主机、其他磁盘和任意个人目录。大小均为逻辑文件长度，未去重，不是压缩后的磁盘占用。扫描期间没有移动、删除或修改研究代码；只在本索引目录写入审计文件。",'',f"当前 720 文件保护清单差异 {len(changed)}；60 条选定参考重新核对路径及 SHA-256；360 份设计 MAT 与控制器 DLL 路径存在性核对。设计大文件没有在本轮全部重新计算哈希。见 {link(OUT/'audit_verification.json')}。",'', '**一、按用途定位**','']
    for group,base,items in GROUPS:
        lines += [f'**{group}**','','| 内容 | 已核对绝对路径 | 状态 |','|---|---|---|']
        for rel,role in items:
            p=Path(base)/rel
            lines.append(f'| {role} | {link(p, str(p))} | {"存在" if p.exists() else "未发现，见核验记录"} |')
        lines.append('')
    lines+=['**二、当前物理文件与引用关系**','',f"60 条实际 MAT 并非都在 registry.json 所在文件夹，逐条实际位置见 {link(OUT/'current_60_reference_paths.json')}；360 份设计及控制器 DLL 见 {link(OUT/'current_360_design_paths.json')}。新控制程序仍引用旧第四章的任务来源/脉冲时钟及旧 RTI 的求解器库，目录名中的旧日期不能用来判定该目录已无用途。",'', '**三、已经删除的历史路径**','', '11 月 23 日已批准并执行删除的 23 个目标包括旧 MPC 系列、ABFD 工作树、deliveries、_archive/20261117 及指定根目录 ZIP/7z。操作回执显示没有新建备份；本次未实施任何删除。根目录 output、outputs、deliverables 与统一工程内 deliverables 仍存在，不能与已删除的 deliveries 混淆。','', '| 历史路径 | 本次现状 |','|---|---|']
    for row in read(OUT/'historical_path_status.json'):
        lines.append(f'| {row["path"]} | {"存在："+row["kind"] if row["exists"] else "不存在"} |')
    lines+=['',f"删除前逐文件清单：{link(ROOT/'cleanup_20261123/delete_groups_3_4_10/manifest.jsonl')}；已批准清单：{link(ROOT/'cleanup_20261123/delete_groups_3_4_10/approved_plan.json')}；核验：{link(ROOT/'cleanup_20261123/delete_groups_3_4_10/verification.json')}。",'', 'stewart-simscape/.git 仍保留 10 个分支引用、5 个工作树登记。已提交代码可以在 Git 中追溯；不能由此推断未提交/未跟踪数据可恢复。ABFD 的 9 月 14 日 ZIP 另存于毕业设计目录，已读取中央目录（3,675 个成员），并不是最后删除状态的完整备份。旧归档文件已不存在，11 月 17 日的 restore.py 和清单本身不能代替归档内容。', '', f"当前工作树与分支：{link(OUT/'worktrees.txt')}，{link(OUT/'branch_refs.txt')}。所有发现的压缩包路径：{link(OUT/'archive_paths.json')}。选定来源包的成员索引：{link(OUT/'selected_archive_members.jsonl')}；本次只读 ZIP 中央目录，未逐包进行 CRC/恢复测试。",'', '**四、完整扫描范围与体量**','','| 扫描目录 / 文件名发现根 | 文件数 | 逻辑 GiB |','|---|---:|---:|']
    for p,v in scope['totals'].items():lines.append(f'| {p} | {v["files"]:,} | {v["logical_bytes"]/2**30:.3f} |')
    lines+=['', '**五、主项目一级目录统计**','','| 目录 | 文件数 | 逻辑 GiB |','|---|---:|---:|']
    for p,v in folders.items():
        if Path(p).parent==U:lines.append(f'| {link(p,str(p))} | {v["files"]:,} | {v["logical_bytes"]/2**30:.3f} |')
    lines+=['', '**六、如何找某一份旧文件**','', '目录概览优先使用本索引；精确到单个文件时，用 search_paths.py 查询保存的完整清单。示例只读清单，不会执行仿真：','', '```powershell',f"& '{U / 'cache/long_payload_v2_20261109/python/Scripts/python.exe'}' -X utf8 '{OUT / 'search_paths.py'}' IHSID --limit 50",f"& '{U / 'cache/long_payload_v2_20261109/python/Scripts/python.exe'}' -X utf8 '{OUT / 'search_paths.py'}' ch5_adaptive REPORT --limit 30",'```','',f"全部目录：{link(OUT/'directories.jsonl')}；目录统计：{link(OUT/'directory_summary.json')}；外围发现：{link(OUT/'discovered_external_paths.json')}；检索范围/联接/错误记录：{link(OUT/'scope.json')}。本索引是扫描时刻的快照，后续文件变化以现场存在性为准。"]
    (OUT/'02_项目路径总索引.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(verification,ensure_ascii=False,indent=2))

if __name__=='__main__':make()
