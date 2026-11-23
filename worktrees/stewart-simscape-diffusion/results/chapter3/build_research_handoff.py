from pathlib import Path
import zipfile,json,hashlib,datetime,shutil
root=Path.cwd();res=root/'results/chapter3';out=res/'diffusion_research_handoff_v1';out.mkdir(exist_ok=True)
files={}
def add(f,dest):
 if f.is_file():files[str(dest).replace('\\','/')]=f
# Current readable source tree, not third-party binaries.
for folder in ['diffusion','chapter3','analysis','opt_minimal','src','matlab']:
 for f in (root/folder).rglob('*'):
  if f.is_file() and f.suffix.lower() in ['.m','.py','.json','.md','.ps1','.txt','.yml','.yaml'] and '__pycache__' not in str(f) and f.stat().st_size<2_000_000:add(f,Path('code_current')/f.relative_to(root))
for folder in ['chapter3_v1','chapter3_v2','chapter3_v3']:
 p=res/folder
 for f in p.glob('*.json'):add(f,Path('evidence/chapter3_legacy')/folder/f.name)
 for f in (p/'report').glob('*'):
  if f.suffix.lower() in ['.json','.csv','.md','.png']:add(f,Path('evidence/chapter3_legacy')/folder/'report'/f.name)
for folder in [p for p in res.iterdir() if p.is_dir() and p.name.startswith('boundary_')]:
 for f in folder.rglob('*'):
  if f.suffix.lower() in ['.json','.html','.png','.csv'] and f.stat().st_size<3_000_000:add(f,Path('evidence/boundary')/f.relative_to(res))
for folder in ['results/remote/diffusion','results/diffusion']:
 for f in (root/folder).rglob('*'):
  if f.is_file() and f.suffix in ['.json','.csv','.png','.md'] and f.stat().st_size<2_000_000:add(f,Path('evidence/early')/f.relative_to(root/'results'))
for name in ['payload_delivery_v1','payload_followup_delivery_v2','payload_validation_delivery_v2','payload_audit_v1','payload_integration_v1']:
 for f in (res/name).rglob('*'):
  if f.is_file() and f.suffix in ['.html','.json','.png','.csv']:add(f,Path('evidence/payload')/name/f.relative_to(res/name))
# Unpack nested snapshots into one navigable package (without recompressing duplicate copies).
for name,prefix in [('payload_validation_complete_v2.zip','evidence/payload_complete'),('payload_handoff_live.zip','evidence/latest_snapshot')]:
 d=out/('_extract_'+Path(name).stem)
 with zipfile.ZipFile(res/name) as z:z.extractall(d)
 for f in d.rglob('*'):
  if f.is_file():add(f,Path(prefix)/f.relative_to(d))
for n in ['payload_source_before_learning_v2.zip']:add(res/n,Path('historical_source_archives')/n)
# User-supplied specifications, if still available.
for ident in ['e8743974-b392-44de-a48c-ede7a43883a8','584e7ea2-5d62-43f1-ad1a-562cf0a42278']:
 f=Path('C:/Users/David_X/.codex/attachments')/ident/'pasted-text.txt';add(f,Path('background/original_requests')/(ident+'.txt'))
live=out/'_extract_payload_handoff_live/payload_formal_references_v1';rs=[json.loads(f.read_text(encoding='utf-8')) for f in live.glob('attempt_*.json')];unique={r['taskId'] for r in rs if r['passed']}
sections=[
('1. 研究起点与路线变化','最初目标是用条件扩散降低 Stewart 平台类似任务反复求解 IHSID 的成本。随后要求统一全时域任务、数值参考、独立物理验收、质量评价和真实耗时。强检索与历史热启动成为必要对照：扩散只快于几何初值优化不足以证明必要性。双工装阶段进一步试图识别规定历史路径适配与重新计时的能力边界；机制证据不足时，不得专门调整场景制造扩散优势。最终转向匀质圆柱／箱体的质量和尺寸变化，分别研究历史复用能力与物理信息对泛化的作用。'),
('2. 最早扩散与引导实验','evidence/early 保存 pilot_v4、formal_v1 和 fastpath_v2 的独立记录。formal_v1 审计记录800次标签尝试、698条接受标签，离线尝试累计117301.69秒。fastpath_v2 是同20个留出任务上的回顾诊断：14任务直接通过、6任务修正通过、没有IHSID回退；不能称为新增独立泛化证据。该版本传统基线、物理准备、任务相关建图和引导协议均有特定定义，详见 code_current/diffusion/README.md 与原审计。'),
('3. 统一第三章旧版实验','chapter3_v3 历史汇总5070次计划调用、5070次完成、4658次达标，含11次运行异常事件。252条件由40独立任务、124相关扫描和88连续序列条件组成，不能合并宣称252种独立场景。旧版采用验证选择的平稳侧重综合权重，主要生成耗时扣除独立评价；当前载荷研究以包含检查的真实调用预算为主，不与旧速度数字混算。历史标签也不全部来自新固定参考目标。完整限制与公式见旧 chapter3_materials.md。'),
('4. 双工装与载荷转向的证据','双工装包中的阶段A、几何探针、参考小试与实现报告为不同时间点快照，不把早期零求解状态覆盖为后来的结果。boundary_delivery_v1 记录72几何候选、12稠密通过，且完整历史复用边界未成立。随后载荷48组合小试：固定路径通过28组合，16组合具有间隙违反见证，6组合力超限，重合2；代表任务12条探针均失败。一次重新计时失败不是不存在可行时间安排的证明，几何探针也不是正式训练库。'),
('5. 当前统一物理模型和判据','固定基座Stewart平台；平台质量50kg，载荷匀质，圆柱轴向x或箱体，固定安装中心[0,0,0.15]m。惯量由形状尺寸质量解析计算，并通过平行轴定理计算合成质心和质心惯量。腿长1.05–2m，腿速0.6m/s，腿加速度1.2m/s²，力±2000N，间隙10mm，归一化最小奇异值≥0.1、条件数≤20。IHSID额外要求Solve_Succeeded、success以及等式／不等式／变量界残差≤1e-6；插值力与重建逆动力学力都检查。硬验收5ms，指定临界与代表轨迹2.5ms。理想化机构包络未经CAD核实；无摩擦、腿惯性、电气及接触模型；数值稠密检查不是连续安全证明。'),
('6. 表示、方法和代码结构','共同表示从16个五次B样条控制点开始，首尾各3固定，60维自由输出。基础条件99维＝93维任务＋形状one-hot2＋包围尺寸3＋质量1；签名255维使总输入354维。M0 ihsid，M1 retrieval（A0/A5/A7/A5-C与固定路径重计时），M2 ddpm_basic，M3 ddpm_signature，M4 ddpm_physloss，M5 ddpm_guided，M6 regression。签名包含5个基准构型的静态力、单位空间加速度响应、间隙与奇异性。物理训练损失与在线引导单独计成本，M5共享M2权重。chapter3/payload_generalization 中 pgMakePayload/pgFinalizeTask、pgCandidates、pgPenalty、pgPlan、pgSolve、pgValidate、pg_learning.py 分别负责模型、候选、可微约束、预算、优化、验收与训练。boundary 是复用层，diffusion/opt_minimal 提供底层工具。'),
('7. 已排查问题与修正身份','旧100步线性日程终步仍保留较大信号，新载荷版本统一cosine。开发早期65训练条件小于batch128，旧模型仅150–200次更新；修正为每轮至少32次更新、早停前至少2000次。cosine终步干净样本反演也会放大误差，新cleanProjection使用仅训练集确定的范围，必须披露而不能称为物理认证。同种子诊断：旧0/64，增加更新但不投影0/64，旧权重只投影12/64，更新＋投影23/64。另修复pgAccept参数计数导致求解证据清空、任务相关精修构图耗尽预算等问题。旧结果不与修正版本混合。Python/MATLAB采样共享噪声误差约8.06e-8；断点恢复和假成功拒绝均有测试。'),
('8. 当前开发结果及不利结论','64验证条件、8基础场景、3共享训练种子；增强对照5376调用。2秒分层等权返回率：基础扩散72.9%、检索65.6%、回归64.9%、签名70.1%、物理损失72.2%、引导70.1%；5秒分别74.0%、69.8%、65.3%、73.3%、72.9%、72.2%。基础扩散2秒140/192成功，其中104直接候选、36公共精修；单候选三种子通过23/12/20，回归25/20/23，故流程优势不等于单候选网络更强。任务等权配对基础扩散−检索2秒差11.98个百分点、95%区间[0,25.53]，5秒差9.375、区间[-5.21,25]；区间仅8场景3种子，不宣称稳健领先。物理增强尚无稳定总预算增益。4次在线接受在2.5ms复查失败，最终已剔除；此前只检查5ms字段的状态汇报漏报，现已纠正。'),
('9. 正在执行的正式阶段',f'本包远程采集快照共有{len(rs)}次参考尝试、{sum(r["passed"] for r in rs)}次严格合格、{len(unique)}个条件至少有一条合格解。任务清单冻结300训练、120验证、192测试；当前仅训练／验证参考解运行，尚无正式模型和正式测试性能。开发测得平均求解35.24秒，据此含25%裕量总参考成本预测44.64小时，72小时上限中为测试预留9.40小时。训练先4初值，不足2模式再至8；验证最多4。包中reference_summary.json可能仍是首批checkpoint，实时阶段应以progress.json与worker status为准。该快照不是持续同步结果。'),
('10. 后续步骤与禁止越界','参考解完成后独立复查、共同表示拟合、模式去重及损坏率检查；再用正式训练／验证数据训练和选参。正式测试仅在协议和来源哈希齐全后运行，不根据测试输赢改域或调参。G0复用载荷原型、G1域内新实例、G2未见组合、G3轻度外推分别报告；基础场景不跨split。包含所有在线处理的B={0.5,1,2,5}s独立调用，主要比较G2、B=2s。冻结的是任务规格，不是已完成全部方法／模型协议。'),
('11. 复核与运行说明','从 code_current 恢复相对目录。MATLAB R2019a、CasADi及相容HSL求解器依赖需另配；MATLAB和HSL商业组件不随包发布。远程曾使用D:/XieZhenyu/envs/stewart-abfd/Scripts/python.exe、RTX4080训练，在线单CPU线程；路径仅为来源记录，应改为本机路径。run_payload.py 提供 audit/make-tasks/references/build-dataset/train/freeze/benchmark/summarize；正式新参考使用pgFormalReferences，调度worker.py位于latest_snapshot。先检查依赖和协议路径再运行，勿直接将旧开发入口当正式参考入口。模型导出含权重但未附完整优化器状态；不承诺本包可原地继续所有训练。'),
('12. 包内容、范围和完整性','本包是完整研究脉络＋核心源码＋关键结果的轻量交接包，不是全量实验磁盘镜像。收录完整本轮开发调用JSON、历史逐任务汇总表、关键图、当前参考尝试JSON、代表MAT轨迹、当前数据JSON和4个开发模型导出。省略全部原始轨迹、所有种子checkpoint、优化器状态、第三方二进制、视频以及重复回收快照。code_current是打包时源码，不伪称各历史实验原始源码；历史身份以各run的codeHash/provenance为准，修正前载荷源码另附存档。所有收录文件的大小与SHA256列于MANIFEST.json，可用verify_package.py校验。')]
import html
body=''.join('<h2>'+html.escape(h)+'</h2><p>'+html.escape(t)+'</p>' for h,t in sections)
page='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Stewart扩散研究完整背景与交接</title><style>body{max-width:1050px;margin:36px auto;padding:0 25px;font:16px/1.85 system-ui;color:#203344}h1{font-size:28px}h2{font-size:21px;margin-top:28px}p{white-space:pre-wrap}a{color:#126b87}</style><h1>Stewart扩散研究：前因、实现、结果与后续边界</h1><p>打包时间：'+datetime.datetime.now().astimezone().isoformat()+'</p><p><a href="evidence/payload/payload_validation_delivery_v2/implementation_report.html">最新开发验证图表</a> · <a href="evidence/chapter3_legacy/chapter3_v3/report/chapter3_materials.md">旧第三章完整论证材料</a></p>'+body+'</html>'
(out/'START_HERE.html').write_text(page,encoding='utf-8');add(out/'START_HERE.html','START_HERE.html')
(out/'BACKGROUND.txt').write_text('\n\n'.join(h+'\n'+t for h,t in sections),encoding='utf-8');add(out/'BACKGROUND.txt','BACKGROUND.txt')
verify="""from pathlib import Path
import hashlib,json
p=Path(__file__).resolve().parent
m=json.loads((p/'MANIFEST.json').read_text(encoding='utf-8'))
for r in m['files']:
 f=p/r['path'];assert f.is_file(),r['path'];assert f.stat().st_size==r['bytes'],r['path'];assert hashlib.sha256(f.read_bytes()).hexdigest()==r['sha256'],r['path']
print('VERIFIED',len(m['files']),'files')
"""
(out/'verify_package.py').write_text(verify,encoding='utf-8');add(out/'verify_package.py','verify_package.py')
# Size budget: retain summaries plus a representative probe, not every dense duplicate.
excluded=[]
for name in list(files):
 if ('boundary_geometric_probe_v1/' in name and files[name].stat().st_size>200000 and files[name].name!='summary.json') or ('payload_basic_validation_v2/calls/' in name) or ('payload_formal_references_v1/attempt_' in name and files[name].suffix=='.mat' and files[name].name not in ['attempt_00001.mat','attempt_00002.mat']):
  excluded.append(dict(path=name,bytes=files[name].stat().st_size,reason='Lightweight budget; summaries and newer full development calls retained'));del files[name]
(out/'EXCLUDED_DETAILS.json').write_text(json.dumps(excluded,ensure_ascii=False,indent=2),encoding='utf-8');add(out/'EXCLUDED_DETAILS.json','EXCLUDED_DETAILS.json')
manifest={'created':datetime.datetime.now().astimezone().isoformat(),'scope':'research handoff; not full raw archive','files':[{'path':n,'bytes':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for n,f in sorted(files.items())]}
(out/'MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8');add(out/'MANIFEST.json','MANIFEST.json')
z=res/'Stewart_diffusion_background_code_results_v1.zip'
with zipfile.ZipFile(z,'w',zipfile.ZIP_DEFLATED,compresslevel=7) as a:
 for n,f in sorted(files.items()):a.write(f,n)
assert z.stat().st_size<50_000_000, z.stat().st_size
with zipfile.ZipFile(z) as a:
 assert a.testzip() is None
 for row in manifest['files']:assert hashlib.sha256(a.read(row['path'])).hexdigest()==row['sha256']
print(json.dumps({'zip':str(z),'bytes':z.stat().st_size,'files':len(files),'referenceSnapshotAttempts':len(rs)},ensure_ascii=False))
