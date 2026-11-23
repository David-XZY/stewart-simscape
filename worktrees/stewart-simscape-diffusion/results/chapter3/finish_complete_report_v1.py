import pathlib,json,hashlib,zipfile,shutil,gzip,re
P=pathlib.Path(__file__).resolve().parent;O=P/'Stewart_complete_report_20260926'
tech='''<!doctype html><meta charset="utf-8"><title>技术细节补充</title><style>body{font:15px/1.8 "Microsoft YaHei",sans-serif;max-width:1000px;margin:35px auto;color:#193143}pre{white-space:pre-wrap;background:#eef4f6;padding:15px}summary{font-size:19px;font-weight:bold;cursor:pointer}details{margin:24px 0}code{background:#edf2f4}a{color:#087f8c}</style><a href="报告.html">正文</a> · <a href="附录.html">完整数据</a><h1>技术细节补充</h1><p>正文解释“为什么”。这里补实现公式、文件入口和复核办法。具体实现以随包源文件为准；这是已经执行版本的快照，不会自动跟随工作站新分支。</p>
<details open><summary>1. 刚体、轨迹和力怎样连接</summary><p>c=(mPcP+mLcL)/(mP+mL)，Π(m,r)=m[(rᵀr)I−rrᵀ]；合成惯量 I=IP+Π(mP,cP−c)+RL IL RLᵀ+Π(mL,cL−c)。姿态为ZYX。每腿上铰点世界位置p+RBi；腿向量p+RBi−Ai给出腿长和方向；空间雅可比把平台线/角速度映射到腿速。刚体所需空间力由惯性、重力、质心和角运动决定，通过JᵀF=W求腿力。质量变化不改变给定位姿的腿长或雅可比，但改变力。</p><p>样条q=Bz，q̇=Dz/T，q̈=DDz/T²；角速度/加速度再经过ZYX旋转运动学映射。固定路径重新计时有q̇=qₛβ、q̈=qₛₛβ²+qₛα，腿量同样链式求导。三种表示为spline、ihsid、path_time_map。时间映射不再拟合成另一条路径冒充不变。</p><p>源：pgMakePayload.m、pgFinalizeTask.m、pgPhysics.m、pg_torch.py；边界目录bdBasis.m、bdReconstruct.m、bdPathFunction.m。</p></details>
<details><summary>2. IHSID究竟优化什么</summary><p>区间数60，61节点+60中点。变量包括位姿/速度X、加速度A、力U=F/1000以及分离平面辅助量S。HS关系为 Xc=(Xk+Xk+1)/2+h(fk−fk+1)/8；Xk+1−Xk=h(fk+4fc+fk+1)/6。每个离散点要求逆动力学平衡、腿长/速/加速度及力界；分离法向单位长度，支持函数限制保证载荷间隙，另有机构包络约束。</p><p>固定baseline参考的偏离按[0.1m,0.1m,0.1m,10°,10°,10°]归一化；腿加速度按1.2 m/s²归一化。系数0.1、0.3、0.4分别乘参考偏离、腿加速度平方、奇异惩罚，再加0.08力变化率积分。源文件给出phi具体形式。奇异值硬门在独立验收，不把它说成已有完整NLP奇异值硬约束。</p><p>缓存键用于结构/物理模型/限值/配置，任务端点、T、固定参考、OBB中心/半尺寸/旋转/掩码、合成质量/COM/惯量、载荷尺寸是数值参数。跨形状重建。当前任务初始力重新计算，只有结构兼容才允许复用对偶量。完整求解残差和变量界都检查，不能只信solver success。</p><p>源：pgSolve.m、pgProblemKey.m、pgBuildProblem.m、dpOptions.m、c3SolverGate.m。</p></details>
<details><summary>3. 四种历史适配与时间下界</summary><p>A0使用新baseline+历史残差；A5/A7做连续路径适配。令ΔR=R0,new R0,oldᵀ，位置先刚体对齐，再加 h(s)[pg,new−p0,new−ΔR(pg,old−p0,old)]；姿态在ΔRRold(s)前乘 Exp(h(s)Log[Rg,new(ΔRRg,old)ᵀ])。h5=10s³−15s⁴+6s⁵，h7=35s⁴−84s⁵+70s⁶−20s⁷；保持连续姿态分支。A5-C另做共同表示拟合；拟合失败不删除原始路径。</p><p>时间映射12个五次控制点，首尾各3个固定0/1，控制点非降；40迭代。离线增强12→16→24属于诊断，不把失败当不存在任何重新计时。沿顺序采样腿长，总变差Σ|Δl|/vmax是必要运动时长下界；等待不能减小它。采样漏掉的变化和数值误差必须单列，未做严格误差界时称数值诊断。</p><p>检索仅访问训练库；端点姿态用SO(3)距离而非简单欧拉角差；尺度来自训练集；本版验证选dimensions_x2。不禁止跨形状检索。当前在线检索并非穷尽全库，检索失败不能推出全库路径都不可用。</p><p>源：pgDistance.m、pgCandidates.m、pgRetime.m、bdAdapt.m、bdTimeLowerBound.m。</p></details>
<details><summary>4. 扩散日程、物理签名、损失和引导</summary><p>标准化输出为60维自由参数；条件归一化只用训练集。前向 zt=√ᾱt z0+√(1−ᾱt)ε；模型预测ε；恢复 ẑ0=(zt−√(1−ᾱt)εθ)/√ᾱt。100步cosine而非旧线性；时间编码是sin/cos。小数据训练提高更新数；预测干净值按训练范围裁剪防数值放大，不能当物理安全措施。</p><p>签名的五点取s=0、.25、.5、.75、1固定baseline构型，不读取当前参考答案。零速度静态力F0；六单位世界空间加速度响应A[:,j]=F(q,0,aj)−F0；角加速度先转换为对应欧拉二阶量。力/响应除2000N，间隙除.01m，σ除.1、κ除20；极端值可截断且带有效标志。σ&lt;10⁻⁴的编码正则化不用于接受。障碍填充槽与基础条件掩码对齐。354维模型的容量控制通过加宽基础模型比较，16控制点对应宽305。</p><p>物理损失Φ为七组 mean(ReLU(vg)²)之和；每组归一化到对应限值，并按有效约束/点平均。不给JᵀF−W加无意义恒等式罚。训练平滑支持函数要保守；33点不替代5ms硬验收。M4只在低噪声1–30步上恢复干净参数计算物理罚；第21–50轮权重爬升，λ验证选.01。</p><p>M5复用M2权重，在20/15/10/5/1步对干净估计做 −η∇Φ/max(RMS∇Φ,10⁻⁸)，控制标准化步长RMS≤.1；最多四次回溯，不下降则保留旧值，再按DDPM后验采样。梯度和回溯均计入在线时间。末端引导对照把更新放到最终样本，区分一般优化作用。</p><p>源：pg_learning.py、pgCandidates.m、pgNoiseSchedule.m、pgPhysicsSignature.m、pgPenalty.m、pg_torch.py。具体存在的文件名可在code目录检索；模型导出/采样、候选种子及归一化断言在同目录。</p></details>
<details><summary>5. 完整验收、质量与计时</summary><p>独立检查按有限值→腿量→力→奇异性→载荷间隙→机构包络→边界进行早拒绝，现有紧凑日志只保留首个失败项。对于载荷，分离间隙不足还会尝试更完整方向优化；依然不足时记clearance_not_certified。它没有自动附带违规上界见证，不能称为碰撞。端点同样使用静态力/腿长/间隙检查。</p><p>公共精修作用于60自由参数，七组物理罚，最多40迭代/四次回溯，固定端点T；质量策略另外加绕路罚。参数尺度位置.1、姿态.2，默认步幅.03，经RMS梯度归一化。改变路径属于精修，不再标成只重新计时。原始失败候选与来源保留，但现有候选元数据未完整绑定每次验收的详细约束，这是报告建议补强的内容。</p><p>路径长度用位置增量，旋转长度用SO(3)旋转量；RMS按统一时间网格；六腿正负行程分开。绝对机械功为∫Σ|Fi l̇i|dt，不能称电能；力RMS/机械功不能替代执行器峰值硬约束。质量门只加绕路，不添加未经应用标定的jerk硬门。</p><p>总时钟覆盖任务初始化到返回组装。归档、离线复查不算在线；求解器内部物理/梯度/约束算求解时间；签名和引导内部物理计算分别算编码/引导。每预算独立运行，迭代检查非硬实时中断，逾期即不合格。零耗时缓存不能灌入任务相关结果。</p><p>源：pgValidate.m、pgEndpointCheck.m、pgAccept.m、pgRepair.m、pgQuality.m、pgPlan.m、pgBenchmark.m。</p></details>
<details><summary>6. 文件说明、可复算范围和缺项</summary><pre>报告.pdf / 报告.html：精简正文；附录.html：全部调用可筛选。
data/main_all_calls.jsonl.gz：20160条，7主方法、5预算。
data/controls_all_calls.jsonl.gz：6912条，6补充方法、2预算。
data/quality_all_calls.jsonl.gz：3024条，48预选任务、3预算。
data/test_reference_attempts.jsonl.gz：768次离线测试参考。
data/historical_chapter3_v3_calls.jsonl.gz：5059条旧版本原始JSON，绝不合并统计。
data/historical_all_jobs.csv：原旧报告5070条总表，含11次运行异常。
data/summary.json：所有方法/预算/G0-G3/总计、耗时和来源。
data/paired_intervals.json：预设及补充配对统计，2000次成簇bootstrap。
data/joint_success_quality.json：双方均成功子集全部质量指标配对。
data/tasks_192.json：冻结任务参数、端点拒绝和参考可行证据。
data/*frozen.json.gz：原预算冻结协议、模型/数据路径与身份。
data/formal_dataset.json.gz：原版本共同表示标签及来源。
evidence：验证选参摘要、审计、失败明细、474条件新分支快照。
code：当前相关MATLAB/Python源码与本报告生成程序。
MANIFEST.json / verify_package.py：逐文件SHA256和校验入口。</pre><p>包不含全部MAT轨迹、模型权重或CasADi/PyTorch/MATLAB环境；trajectoryRef指向原工作站。原版本源轨迹库的清单不等于轨迹文件已嵌入。当前无法回收的正式训练/验证逐次日志只提供已存在摘要，不用旧快照填充冒充完整。全库排除诊断、完整12案例、CVAE/库规模实验没有可交付的完整结论，不在图表中补造。</p><p>重新计算表：按scope、method、B、generalizationType分组，以finalQualified求均值；不能用onlinePassed替代。bootstrap按24个baseSceneId重抽，种子区组对所有场景共同重抽。30096是调用数，192是任务数，24才是基础场景簇数。旧预算和10秒分别冻结、分别真实执行；不要把10秒分析追认成原预设主结论。</p></details>'''
(O/'技术细节.html').write_text(tech,encoding='utf-8')
# Add a visible route to the technical supplement without changing PDF pagination.
f=O/'附录.html';s=f.read_text(encoding='utf-8');s=s.replace('<h1>完整数据附录</h1>','<h1>完整数据附录</h1><p><a href="技术细节.html">展开公式、算法与源文件说明</a></p>');f.write_text(s,encoding='utf-8')
shutil.copy2(P/'chapter3_v3/report/all_jobs.csv',O/'data'/'historical_all_jobs.csv')
shutil.copy2(P/'chapter3_v3/report/runtime_incidents.csv',O/'data'/'historical_runtime_incidents.csv')
shutil.copy2(__file__,O/'code'/pathlib.Path(__file__).name)
shutil.copy2(P/'render_complete_report_pdf.cjs',O/'code'/'render_complete_report_pdf.cjs')
verify='''import pathlib,json,hashlib,gzip
p=pathlib.Path(__file__).resolve().parent
m=json.loads((p/'MANIFEST.json').read_text(encoding='utf-8'))
for name,d in m['files'].items():
 f=p/name
 assert f.stat().st_size==d['bytes'],name
 assert hashlib.sha256(f.read_bytes()).hexdigest()==d['sha256'],name
for name,n in m['rowCounts'].items():
 with gzip.open(p/'data'/name,'rt',encoding='utf-8') as f:assert sum(1 for _ in f)==n,name
print('PASS: all files and archived record counts match.')
'''
(O/'verify_package.py').write_text(verify,encoding='utf-8')
manifest={'title':'Stewart complete report 2026-09-26','formalCalls':30096,'historicalVersionSeparate':True,'remoteStatus':'not refreshed due to SSH host identity mismatch; report uses recovered completed logs','rowCounts':{'main_all_calls.jsonl.gz':20160,'controls_all_calls.jsonl.gz':6912,'quality_all_calls.jsonl.gz':3024,'test_reference_attempts.jsonl.gz':768,'historical_chapter3_v3_calls.jsonl.gz':5059},'files':{}}
for f in sorted(O.rglob('*')):
 if f.is_file() and f.name!='MANIFEST.json':manifest['files'][f.relative_to(O).as_posix()]={'bytes':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()}
(O/'MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
zpath=P/'Stewart_complete_report_20260926.zip'
with zipfile.ZipFile(zpath,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
 for f in sorted(O.rglob('*')):
  if f.is_file():z.write(f,f.relative_to(O))
assert zpath.stat().st_size<50_000_000
with zipfile.ZipFile(zpath) as z:assert z.testzip() is None
print(json.dumps({'zipBytes':zpath.stat().st_size,'sha256':hashlib.sha256(zpath.read_bytes()).hexdigest(),'files':len(manifest['files'])},ensure_ascii=True))
