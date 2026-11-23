"""Render a compact Chinese report and a self-contained numeric appendix."""
import pathlib,json,gzip,html,re,hashlib,zipfile,shutil
P=pathlib.Path(__file__).resolve().parent;O=P/'Stewart_complete_report_20260926'
def read(p):return json.loads(p.read_text(encoding='utf-8'))
S=read(O/'data/summary.json');C=read(O/'data/paired_intervals.json');Q=read(O/'data/joint_success_quality.json')
M=['ihsid','retrieval','regression','ddpm_basic','ddpm_signature','ddpm_physloss','ddpm_guided']
L=dict(zip(M,['IHSID','增强检索','普通回归','基础扩散','物理签名扩散','物理损失扩散','物理引导扩散']))
L.update(ddpm_basic_wide='加宽基础扩散',ddpm_signature_no_accel='签名去加速度响应',ddpm_signature_no_gap='签名去间隙',regression_signature='签名回归',retrieval_signature='签名检索',ddpm_end_guided='末端梯度修正')
def st(m,b,g='ALL',scope='main'):return next(r for r in S if (r['method'],r['B'],r['group'],r['scope'])==(m,b,g,scope))
def table(head,rows):return '<table><thead><tr>'+''.join('<th>'+str(x)+'</th>' for x in head)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+str(x)+'</td>' for x in row)+'</tr>' for row in rows)+'</tbody></table>'
def pct(x):return f'{100*x:.1f}%'
def rate(m,b,g='ALL',scope='main'):
 r=st(m,b,g,scope);return f"{r['qualified']}/{r['n']}<small>{pct(r['rate'])}</small>"
def ci(m,base,b,g='G2'):
 r=next(c for c in C if (c['method'],c['baseline'],c['B'],c['group'])==(m,base,b,g));return f"{100*r['difference']:+.2f} [{100*r['ci95'][0]:+.2f}, {100*r['ci95'][1]:+.2f}]"
pages=[]
def page(title,body,kicker='研究报告 · 2026-09-26'):
 pages.append(f'<section class="page"><div class="kicker">{kicker}</div><h1>{title}</h1>{body}<footer>Stewart 匀质载荷轨迹规划｜原始正式版本＋10秒追加实验｜{len(pages)+1}</footer></section>')
page('这项研究做了什么，结果怎样', '''
<p class="lead">让六条电动腿托着不同的圆柱或箱体，在规定时间内绕开障碍，从起点移动到终点。我们比较：每次重新优化、借用历史轨迹、普通神经网络预测，以及扩散模型生成。核心问题是<b>同样的在线时间，谁能交付更多真正合格的轨迹</b>。</p>
<div class="callout"><b>当前结论：</b>2秒预设主比较没有证实基础扩散优于增强检索；10秒追加实验出现可行返回率收益。但扩散的成功轨迹在若干平滑性指标上更差，不能写成“没有质量代价的全面提升”。</div>
<h2>为什么研究方向发生变化</h2>'''+table(['阶段','实际做了什么','留下的结论'],[
['旧第三章统一实验','5070次调用，4658次合格；统一目标、物理门、评分与计时','场景、预算和评分不同，不能与本报告成功率混算。'],
['固定载荷、双工装边界','72条几何候选中12条通过；同一条路径在25个偏移任务通过','没有建立“必须生成新路径”的充分证据，增强复用可能已足够。'],
['改变载荷的小试','48组质量/尺寸/时长；同一路径通过28组，16组有间隙违反见证、6组力超限，重合2组','载荷变化确实使旧路径失效；当时还不能证明扩散能解决。'],
['本次正式载荷研究','同数据训练、冻结测试；原预算21600次，追加10秒8496次','现在可以比较合格返回率、质量、耗时和失败原因。']])+'''
<h2>先记住三个区别</h2>
<p><b>运动时长 T</b>是平台实际走完动作的时间；<b>规划预算 B</b>是电脑想办法的时间。把B从2秒加到10秒，没有自动放慢动作。</p>
<p><b>物理合格</b>是约束不过界；<b>质量好</b>是路径、加速度、jerk和力等表现更好。通过前者不保证后者。<b>没有找到解</b>也不等于任务无解。</p>
<h2>怎样阅读</h2>
<p>第2–4页解释模型、算法和数据；第5–8页看结果与失败；第9页给改进顺序；第10页给复核入口和边界。网页附录可按任务、方法、预算筛查全部30096次载荷调用，原始记录随包提供。</p>
<p class="note">状态边界：本报告数值来自已完成并回收的原版本。新474条件分支单列在第9页；当前远程身份校验失败，不能把本地训练快照冒充实时进度。</p>''')
page('问题和物理模型：到底约束了什么', '''
<p>任务输入是起终位姿、固定T、最多6个障碍框，以及载荷形状、尺寸、质量。全程保留六个运动自由度，没有强制途径点。原论文两阶段任务和第四章控制器不在本轮修改范围。</p>'''+table(['固定内容','取值/含义'],[
['机构','基座/动平台铰点半径0.75/0.50 m；支链映射[6,1,2,3,4,5]。'],
['平台','50 kg；平台中心惯量diag(3.125,3.125,6.25) kg·m²。'],
['载荷','匀质圆柱（轴沿局部x）或箱体；几何中心就是自身质心；固定安装偏置[0,0,0.15] m。'],
['开发域','质量50–400 kg；长度0.8–1.3 m；圆柱半径0.10–0.15 m；箱宽0.20–0.30 m、高0.16–0.28 m。'],
['环境','开放转移/双工装；平移、姿态、联合任务。名义T为2、3、5、7.5秒，端点扰动与工装偏移预先生成。'],
['理想化包络','动平台圆柱半径0.55 m、半高0.05 m；腿胶囊半径0.06 m；基座半径0.80 m、半高0.10 m。未由CAD核实。']])+'''
<h2>质量和尺寸怎样进入物理计算</h2>
<p>圆柱惯量：I<sub>L</sub>=diag(mR²/2, m(3R²+L²)/12, m(3R²+L²)/12)。箱体惯量：I<sub>L</sub>=m·diag(Lᵧ²+Lz²,Lₓ²+Lz²,Lₓ²+Lᵧ²)/12。合成质量M=50+m，质心c=m[0,0,0.15]/M；各刚体惯量用平行轴公式移到合成质心后相加。<b>因此变重会改变合成质心和所需力；改变尺寸同时改变碰撞外形与惯量。</b></p>
<p>姿态按R=Rz(ψ)Rᵧ(θ)Rₓ(φ)，rad为单位。角速度、角加速度、角jerk由旋转运动学计算，不把欧拉角导数直接当角运动量。逆动力学用当前载荷、位姿、速度、加速度重新计算六腿力，不缩放旧力冒充新力。</p>
<h2>所有方法共用的硬门槛</h2>'''+table(['检查','要求'],[
['边界/数值','有限值；固定T；起终位姿及静止速度、加速度正确，边界误差≤10⁻⁶。'],
['环境间隙','载荷及声明机构包络全程/终点≥10 mm。'],
['执行器','腿长1.05–2.00 m；腿速≤0.60 m/s；腿加速度≤1.20 m/s²；驱动力−2000～2000 N。'],
['奇异性','归一化J·diag(1,1,1,2,2,2)的最小奇异值≥0.1，条件数≤20。'],
['IHSID额外三重门','success且Solve_Succeeded、未超时；等式/不等式/变量界残差均≤10⁻⁶；独立物理检查通过（含插值力和逆动力学力）。']])+'''
<p class="note">模型包含刚体惯性和重力，不含腿惯量、摩擦、电气、接触、安装自碰撞或跟踪误差。本文结论是标称仿真规划能力，不是真实整机安全认证。</p>''')
page('算法怎样走：先生成，再检查和补救', '''
<p><b>统一轨迹表示：</b>每个自由度16个五次B样条控制点，首尾各3个固定，剩10×6=60个待生成数。固定首尾三点使端点速度、加速度为零。原始IHSID轨迹仍保留，检索另保留不强制拟合的路径。</p>'''+table(['方法','大白话原理','实际输入/处理'],[
['M0 IHSID','把全部运动和力一起优化','60个Hermite–Simpson区间，节点/中点共121组状态；CasADi＋IPOPT/MA27。'],
['M1 增强检索','找相似任务的历史路径再适配','7块距离：位置、SO(3)姿态、障碍、log T、形状、尺寸、log质量；验证选尺寸权重×2。'],
['M2 基础扩散','从噪声反复去噪，生成候选','99维条件：旧任务布局93维＋形状2维＋包围尺寸3维＋质量1维；100步cosine日程。'],
['M3 物理签名','先探测当前任务的力和空间，再生成','五个固定baseline构型，每点静态力6、加速度响应36、间隙6、奇异性2、有效标志1；255＋99=354维。'],
['M4 物理损失','训练时就罚违规轨迹','仍是99维；预测干净控制点后重建33点轨迹，对7组归一化违规平方加罚。'],
['M5 物理引导','去噪中插入少量物理梯度修正','复用M2权重，在20/15/10/5/1步引导；验证η=0.03，RMS更新≤0.1，最多4次回溯。'],
['M6 普通回归','直接预测一个确定轨迹','99维输入；每条件选择原优化目标最低的标签，避免把不同解平均；只产生1候选。']])+'''
<h2>候选方法共用的在线流程</h2>
<div class="flow">端点检查 → 编码/检索/生成 → 批内粗排 → 完整验收<br>未通过 → 最多2候选轻量修正 → 候选热启动IHSID → 返回或失败</div>
<p>累计候选数为1、4、8、16、32、64，不重复生成；候选编号绑定随机流。按33点粗查违规量排序，并列按编号；逐条完整检查，首条合格立即返回。粗排不算验收。</p>
<p>候选阶段到总预算50%，公共修正到75%，最后25%留给IHSID回退；提前结束便推进，不空等。每候选最多40次修正，回退最多8初值。M0把全预算用于自己的求解及验收。超时后才完成的结果一律失败。</p>
<h2>检索不是简单复制</h2>
<p>A0把历史残差叠到新baseline；A5/A7先刚体对齐，再用五次/七次平滑函数修正端点；A5-C另拟合共同样条。随后可用12控制点单调五次时间映射重新安排快慢，最多40迭代。路径hash保持不变；形状修正另记阶段。重新计时失败只表示本实现没找到解。</p>
<p class="note">本轮离线参考采用固定目标和初值序列。IHSID是高质量数值参考，不保证全局最优；扩散直接、内部引导、公共修正及回退的贡献分别记录。</p>''')
page('实现、训练和数据：数量不能混着数', '''
<h2>参考解与标签</h2>
<p>原版本预定300训练、120验证、192测试条件，按基础场景隔离；同场景派生任务不跨集合。训练/验证共2880次参考尝试，1576次严格合格；去重、共同表示拟合后，只有<b>175个训练条件＋53个验证条件</b>有可用标签，共228条。拟合损坏率4.44%，低于5%扩表示触发线，所以保留16控制点。评价仍保留全部120验证和192测试条件，包括非法/失败任务。</p>
<p>测试离线参考另做768次尝试：401次严格合格，覆盖110个测试条件。它们只用于分析，未进入训练。每次上限120秒，默认baseline、正偏航、负偏航、抬升；训练可追加到8初值。</p>
<h2>网络和训练细节</h2>
<p>MLP/DDPM均为3层256宽tanh；AdamW学习率10⁻³、batch128、最多300轮、单次最多6小时。三种子20260919/20/21。先等概率抽条件，再抽标签；每10轮固定24验证条件做真实验收，以合格率优先、预测损失次之选检查点。修正后每轮至少32次更新，早停至少2000次更新；30轮无改善停止。</p>
<p>cosine偏移0.008、β最大0.999，保存完整日程并对齐MATLAB采样。采样对预测干净参数采用仅由训练标签确定的标准化范围裁剪；这是数值稳定处理，不是安全保证。M4每批32条样本在第1–30噪声时刻计算33点物理损失，前20轮不用物理罚，第21–50轮线性增加；验证从0.01/0.1/1选0.01。物理核float64、网络float32，梯度贯通。M34组合未触发预定条件，未开展正式比较。</p>
<h2>修过哪些会影响结论的问题</h2>'''+table(['问题','处理与边界'],[
['旧93/96维条件未贯通','统一99/354维schema；模型、数据、归一化、协议身份严格校验。'],
['旧线性噪声末端仍残留信号','100步旧日程ᾱ≈0.36356；改cosine，所有新扩散统一重训。'],
['小数据每轮更新太少','增加每轮更新下限；记录实际优化步数，不能只报epoch。'],
['求解门/空候选错误','修复验收参数覆盖及空池循环；正式v1仅6条受影响记录保留，全部方法以v2重跑。'],
['重复构图、嵌套计时','物理核/NLP用任务数值参数；按结构缓存；互斥计时并核对闭合。']])+'''
<p class="note">早期交叉检查：三类载荷各1000状态，MATLAB/CasADi与PyTorch力相对误差最大约3.65×10⁻¹⁵，方向梯度误差约1.52×10⁻⁹。是特定测试集的数值一致性证据，不代表全部输入无误。原文件与版本保留在evidence。</p>''')
page('结果一：更多预算带来多少成功', '''
<p>每种方法每个预算均为192任务×3区组=576次真实调用。三次重复不增加独立任务数。G0为新任务/已见载荷原型，G1为域内新载荷，G2为未见组合，G3为质量或长度轻度外推；各48任务、24基础场景。10秒由用户追加，不能替换原先G2、2秒主比较。</p>'''+table(['方法','0.5秒','1秒','2秒','5秒','10秒'],[[L[m]]+[rate(m,b) for b in [.5,1,2,5,10]] for m in M])+'''
<img class="chart" src="figures/budget.png">
<p><b>读数：</b>增强检索从2秒起停在363/576；基础扩散从362升到392，签名扩散从376升到401。IHSID在2秒内没有最终合格返回，不代表算法不能解，而是该实现的完整求解与验收来不及。</p>
<p>46个端点拒绝任务占23.96%，使这些冻结任务按现有门槛的通过率上限只有146/192=76.04%。保留它们是为了不删失败样本；另看端点通过子集：10秒检索82.88%、基础扩散89.50%、签名扩散91.55%。这是补充条件统计，不覆盖原主结果。</p>
<p class="note">数值单位均为“合格调用/全部调用”。线上通过但离线复查失败已从分子扣除。各预算独立实际调用，不是从10秒运行中截取内部首解时间。</p>''')
page('结果二：哪些优势站得住', '''
<p>用同任务配对，并按基础场景成簇bootstrap 2000次；三个训练种子作为跨场景共享区组。表中为成功率差（百分点）及95%区间。区间跨0，不能排除无差异。</p>'''+table(['比较','G2，2秒：预设','G2，10秒：追加'],[[L[m]+'−'+L[b],ci(m,b,2),ci(m,b,10)] for m,b in [('ddpm_basic','retrieval'),('ddpm_signature','ddpm_basic'),('ddpm_physloss','ddpm_basic'),('ddpm_guided','ddpm_basic')]])+'''
<p><b>准确结论：</b>2秒基础扩散−检索为−1.39个百分点，证据不足。10秒G2下，签名扩散比基础扩散高7.64个百分点，区间[1.39,16.67]；比检索高12.50，区间[2.78,25.00]。全部测试合并，基础扩散−检索为+5.03，[0.52,11.12]。这些是追加/分层分析，未作多重比较校正，仅3个训练种子，不能写成普遍定律。</p>
<h2>补充对照：复杂特征并非越多越好</h2>'''+table(['版本（各576次）','2秒','10秒'],[[L[m],rate(m,2,scope='controls'),rate(m,10,scope='controls')] for m in ['ddpm_signature_no_accel','ddpm_signature_no_gap','ddpm_basic_wide','regression_signature','retrieval_signature','ddpm_end_guided']])+'''
<p>去掉加速度响应的签名扩散达到72.22%，高于完整签名69.62%。加宽基础扩散68.92%也接近完整签名。说明<b>不能将完整签名的点估计收益全部归因于物理信息</b>；还需要辨别特征冗余、容量、归一化和随机波动。签名检索没有增加成功数；末端修正与去噪中引导接近，没有确立去噪引导的独有收益。</p>
<h2>分组10秒结果</h2>'''+table(['方法','G0','G1','G2','G3'],[[L[m]]+[rate(m,10,g) for g in ['G0','G1','G2','G3']] for m in ['retrieval','regression','ddpm_basic','ddpm_signature','ddpm_physloss','ddpm_guided']])+'''
<p class="note">补充方法的正式测试结果不用于追选原主方法；下一版若据此调整，必须明确是新一轮假设，并保留独立确认数据。</p>''')
page('结果三：成功来自哪里，质量有没有代价', '''
<p>下表全部为10秒预算；平均耗时包含成功、失败、端点快速拒绝，不能单独当作“求一条成功解的速度”。首次发现、最终返回和各阶段时间均保存于原始记录。硬截止由迭代回调执行，所以失败可能略超时，超出的输出没有计作成功。</p>'''+table(['方法','直接/内部引导','公共精修','IHSID/回退','平均总秒','P95总秒'],[[L[m],st(m,10)['stageCounts'].get('direct',0),st(m,10)['stageCounts'].get('repair',0),st(m,10)['stageCounts'].get('ihsid',0)+st(m,10)['stageCounts'].get('ihsid_fallback',0),f"{st(m,10)['returnMean']:.3f}",f"{st(m,10)['returnP95']:.3f}"] for m in M])+'''
<p>签名扩散401次成功中366次在公共精修前返回，35次靠公共精修；没有IHSID回退成功。基础扩散为359＋33。M5“直接”已包含去噪内部物理引导，不能称纯网络输出。</p>
<h2>物理通过，不等于轨迹一样好</h2>
<p>只在双方均成功的同任务、同区组上配对，避免不同成功集合混比。下面是“扩散−检索”的<b>配对差中位数</b>，正数表示该负担更大；不是两列中位数简单相减。</p>'''+table(['10秒物理策略','基础扩散（358对）','签名扩散（357对）'],[
['位置路径长度','+7.25 mm','+13.61 mm'],['姿态旋转路径','+0.0120 rad','+0.00964 rad'],['平台线jerk RMS','+1.300 m/s³','+1.423 m/s³'],['力RMS','−0.091 N','+2.613 N'],['绝对机械功','+21.67 J','+24.19 J']])+'''
<div class="callout"><b>必须保留的负面结果：</b>当前扩散可行率收益伴随更大的jerk、绕路和机械功点估计。不能据此宣称“不是质量换时间”。目前没有应用给定的jerk硬阈值，因此这些输出仍可能物理合格，但工程品质需要改进。</div>
<h2>另跑的质量停止策略</h2>
<p>预选48任务×3=144次/方法/预算，附加门只限制位置/姿态绕路。10秒检索102/144，基础扩散103/144，签名扩散103/144，物理损失102/144，引导101/144，回归94/144，IHSID63/144。这个子集不能与全192任务物理率横比，也没有证明jerk更好。</p>
<p class="note">所有原始质量项和共同成功配对差均在附录：位置/旋转路径、腿正负及总行程、平台线角/腿加速度与jerk RMS、力峰值/RMS/变化率、间隙、奇异性、利用率、固定参考代价、机械功。机械功不是电能。</p>''')
page('失败到底卡在哪里', '''
<p>已逐条核对全部10秒调用。主实验7方法×576=4032次：端点拒绝966次；进入规划后最终超时545次；在线通过但加密复查拒绝3次；合格2518次。这里是互斥分类，和候选内部多种违规原因不能相加。</p>'''+table(['方法','端点拒绝','最终超时','复查拒绝','总失败'],[[L[m],st(m,10)['boundaryRejected'],st(m,10)['timeout'],st(m,10)['postAuditRejected'],576-st(m,10)['qualified']] for m in M])+'''
<h2>端点、候选、求解器是三层原因</h2>
<p><b>端点：</b>32任务腿长越界，14任务端点间隙未获证书；各方法各重复3次，都是138次拒绝。前者在固定端点条件下无法靠换中间路径解决；后者还可能受保守几何检查影响。</p>
<p><b>候选：</b>签名扩散37次非端点失败中，34次出现间隙未获证书、23次出现腿运动违规、19次出现力超限；不同候选原因可重叠。日志只记第一项拒绝门，腿运动没有细分长/速/加速度，因此这些不是完整约束归因。</p>
<p><b>求解器：</b>上述主实验超时分支的IHSID/回退状态为User_Requested_Stop; task_wall_budget。含义是到实际预算被程序叫停，不是“证明无解”。如某次停止时等式残差0.119、不等式违反0.479，远高于10⁻⁶，但汇总数不能告诉我们卡在何种物理约束。</p>
<h2>“间隙未获证书”不等于碰撞</h2>
<p>分离平面给出真实距离下界：下界≥10 mm才能接受；下界不足时，真实距离可能也不足，也可能是估计保守。只有实体内点对的距离上界&lt;10 mm，才能确认违反安全间隙（仍不一定接触）。目前失败码没区分“确认违反”和“未知”。3条主实验加密失败复查均为间隙未获证书。</p>
<h2>还有真正未找到任何解的任务吗</h2>
<p>146个端点通过任务中，主方法至少解决143个，加入补充对照解决145个。剩余<b>formal_test_013_box_01</b>在线全部失败，但离线已有16.59秒尝试得到的严格合格解。另两个主方法全失败任务013_box_03、019_cylinder_02被签名消融/签名回归解决。<b>合并全部证据，146个都有可行轨迹；不是无解证明问题，而是各方法的预算、覆盖和验收问题。</b></p>''')
page('下一步改什么，按什么顺序做', '''
<p>优先让失败原因可定位、让成功轨迹更好，再决定是否扩大网络。以下是研究建议；不能用正式测试反复调到满意后仍把它当未见测试。</p>'''+table(['优先级','具体改动','怎样判断有效'],[
['P0 失败可解释','保存每候选ID、首个及全部违反项、时间点/腿号/超限量；求解约束分组残差、迭代轨迹、剩余预算；精修前后罚函数与停止理由。','失败能落到某腿/某障碍/某时刻；区分初值差、物理违规、检查保守、算力截止。'],
['P0 几何判定','对unknown补点对上界见证及更精确距离；记录上下界和数值容差；保留原10 mm门槛。','量化确实违规与保守误拒各占多少；不能靠放松门槛提高分数。'],
['P1 工程质量','在生成/末端修正增加统一平滑目标；评估jerk、腿加速度、力变化率与绕路的约束/代价权衡。','共同成功任务上的配对品质改善，同时不明显损失可行返回率；应用阈值先定后测。'],
['P1 特征与数据','验证更小签名、按块归一化、容量匹配；扩充困难但可行条件，避免把重复优化解当多模式。','仅验证集选方案；完整签名须胜过去特征和加宽控制，才支持特征贡献。'],
['P1 时间分配','分析检索是否被少数重新计时尝试占满预算；验证更多历史候选先粗筛、异步截止/验收预留、短预算回退是否值得。','比较真实返回时间、候选覆盖、修正增量；不能从内部首解时间模拟新预算。'],
['P2 机制与外推','补12个预定机制案例、全库适配/时间下界诊断、数据量和必要的CVAE对照。','才能回答历史路径是否真不够、多模态是否必要、扩散形式是否有独有价值。']])+'''
<h2>已发生的新数据工作，不能与旧成绩混用</h2>
<p>9月26日本地归档显示：多初值合格样条候选1506条全部复查通过，但175个训练条件的4445个解对绝大部分几乎重合；把模式阈值从0.15收紧到0.005只多收6条训练标签。<b>优化次数多不等于数据模式丰富。</b></p>
<p>随后从175条件扩到<b>474个可训练条件</b>：恢复历史36个、新条件接受263个（含时间缩放恢复24个），合计175＋36＋263；验证标签仍53个。另有23个原始轨迹物理合格但共同表示不合格，不能硬塞入网络；总物理接受497条件。</p>
<p>新分支有单独实现/训练快照：batch64、每轮64次更新、12个目标模型、扩展验证面板、无IHSID回退。部分新增T达到10–15秒，是<b>放慢动作</b>，不是把B增加到10秒。旧版本报告的成绩不能当474版成绩；新分支尚无本报告可核实的最终比较。</p>
<p class="note">新分支改了数据和流程，未来需同时报告“仅扩数据”与“流程改变”的贡献。已经看过的192测试条件只能继续作明确标注的复现/诊断；确认性泛化结论宜预留新的冻结场景。</p>''')
page('复核所需的技术口径与数据入口', '''
<h2>容易被省略、但影响结论的细节</h2>'''+table(['项目','实际实现/复核方式'],[
['IHSID目标','固定baseline参考，归一化位姿偏离系数0.1、腿加速度平方0.3、奇异惩罚0.4、力变化率0.08（力决策量以kN计）；节点/中点采用HS积分权重，换初值不换目标。奇异性仍是惩罚＋后验硬门。'],
['离散/稠密','HS约束状态插值、积分及JᵀF=W；离散残差决定求解门，稠密动力学残差仅诊断。碰撞NLP内部另加最高1 mm的sin²余量，最终门仍10 mm。'],
['物理罚/引导','7组：腿长、速、加速度、力、载荷及机构间隙、最小奇异值、条件数；组内均值/组间等权，ReLU违规平方。不添加同一逆动力学定义下恒等的动力学残差损失。'],
['重新计时','q̇=qₛβ，q̈=qₛₛβ²＋qₛα；s(t)单调、固定T与静止边界，重新算力。腿总变差/v上限只是必要时间下界，采样与数值误差下不能冒充连续严格认证。'],
['几何支持','圆柱h(n)=L|nₓ|/2＋R√(nᵧ²＋nz²)；箱体h(n)=ΣLᵢ|nᵢ|/2。训练/修正可用保守平滑近似；硬验收独立，低罚值不是证书。'],
['质量门','只附加绕路：长度/端点距离（分母至少0.01 m），旋转路径/端点旋转（至少1°）；验证条件等权P95×1.5。圆柱上限1.50195/1.90344；箱体1.59275/17.32186。旧综合分不用于本轮主结论。'],
['验收网格','全成功输出独立5 ms复查；代表、间隙余量&lt;2 mm、利用率&gt;95%及固定hash抽10%其他成功增做2.5 ms；在线还对近障碍区加密。加密失败扣除最终成功。不是连续时间证明。'],
['硬件与缓存','ws单规划进程、固定单线程CPU推理，RTX4080用于训练；方法交错。模型/结构图可预热，任务相关签名、排序、适配不可预灌；结构键含机构/形状/限值/离散/求解配置。'],
['耗时/统计','总时间含编码、签名、检索、生成、引导、排序、检查、重新计时、精修、构图/求解、返回及其他；分项互斥，闭合误差最大约7.1×10⁻¹⁵ s。离线审计/写盘另记。']])+'''
<h2>交付内容及“全部数据”的边界</h2>
<p><b>附录.html</b>可筛查30096次已完成载荷调用、192任务、全部预算分组、95%区间和质量配对；<b>data/*.jsonl.gz</b>保存这些调用的原始完整JSON字段，以及768次测试参考日志、旧版本5070条总表及5059条原始JSON；<b>evidence</b>保存失败审计、物理交叉检查、新数据快照；<b>code</b>保存当前实现和分析脚本。</p>
<p>为保持包小于50 MB，未放入全部MAT轨迹、模型权重与第三方运行库；每条调用保留原trajectoryRef和数据/模型/协议身份字段。旧训练验证参考尝试、尚未回收文件及未完成实验不伪装成完整附件。这个包可复算统计，不声称脱离原工作站就能重跑全部规划。</p>
<p>数值来源：payload_heldout_*_v2、payload_heldout_*_budget10_v1；统一数值代码身份65b808d1…8fd24d，原训练数据文件SHA256为396b82e2…82361。完整哈希在原始日志/冻结协议。生成脚本和逐文件SHA256清单随包，方便复算与检查。</p>
<div class="callout">能够写进当前结论：扩散在10秒追加预算的一些范围内提高可行返回率；收益没有覆盖所有预算，也没有消除质量代价。下一步最有价值的是补齐违规归因、改善平滑性、审计签名和数据覆盖，再做独立确认。</div>''')
css='''@page{size:A4;margin:13mm 14mm 12mm}*{box-sizing:border-box}body{font:14px/1.62 "Microsoft YaHei","Segoe UI",sans-serif;color:#193143;background:#eef2f5;margin:0}.page{position:relative;max-width:940px;min-height:1100px;margin:24px auto;background:white;padding:36px 44px 55px;box-shadow:0 2px 12px #ccd5dd}.kicker{font-size:11px;color:#617f93;letter-spacing:2px}h1{font-size:25px;line-height:1.35;margin:12px 0 18px;border-bottom:3px solid #087f8c;padding-bottom:12px}h2{font-size:17px;margin:17px 0 7px;color:#087680}p{margin:9px 0}.lead{font-size:16px}table{border-collapse:collapse;width:100%;font-size:12px;line-height:1.5;margin:12px 0}th{background:#e5f2f3;color:#135867}th,td{border-bottom:1px solid #d4dfe5;padding:7px 8px;text-align:left;vertical-align:top}tr:nth-child(even){background:#f7f9fa}small{display:block;color:#576d7c}.callout{border-left:4px solid #087f8c;background:#eef7f7;padding:12px 15px;margin:14px 0}.note{font-size:12px;color:#526a7b}.flow{background:#e5f2f3;padding:13px;text-align:center;font-weight:600}.chart{width:100%;display:block;margin:12px 0}footer{position:absolute;bottom:17px;color:#6c808b;font-size:10px}a{color:#006d84}.nav{max-width:940px;margin:25px auto;padding:0 15px}.nav a{margin-right:20px}@media print{body{background:white;font-size:13px;line-height:1.55}.page{width:auto;max-width:none;min-height:0;height:267mm;margin:0;padding:0 0 10mm;box-shadow:none;break-after:page;overflow:visible}.page:last-child{break-after:auto}h1{font-size:21px;margin:8px 0 12px;padding-bottom:9px}h2{font-size:15px;margin:12px 0 6px}.kicker{font-size:9px}.lead{font-size:14px}p{margin:7px 0}table{font-size:11px;line-height:1.45;margin:9px 0}th,td{padding:5px 6px}.callout{padding:9px 12px;margin:10px 0}.note{font-size:9px}.chart{margin:8px 0;max-height:80mm;object-fit:contain}.flow{padding:10px}footer{bottom:0;font-size:8px}.nav{display:none}.page:nth-of-type(6) small{display:inline;margin-left:4px}tr,img{break-inside:avoid}}'''
doc='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Stewart扩散轨迹研究完整报告</title><style>'+css+'</style><div class="nav"><a href="附录.html">打开完整数据附录</a><a href="报告.pdf">下载PDF正文</a></div>'+''.join(pages)+'</html>'
(O/'报告.html').write_text(doc,encoding='utf-8')
# Numeric appendix: every call is visible without a Python environment.
compact=[]
for scope in ['main','controls','quality']:
 with gzip.open(O/'data'/(scope+'_all_calls.jsonl.gz'),'rt',encoding='utf-8') as f:
  for line in f:
   r=json.loads(line);compact.append([scope,r['taskId'],r['generalizationType'],r['method'],r['B'],r['repeat'],int(r['finalQualified']),round(r['actualReturnTime'],6),r['reason'],r['returnStage'],r['candidateGenerated'],r['candidateChecked'],r['runId'],dict(__import__('collections').Counter(a.get('reason','') for a in r['attempts'])),r.get('rawMetrics',{})])
script='''let page=0;const n=100;let shown=[];const fields=['组','任务','分组','方法','预算秒','区组','最终合格','实际秒','终止原因','返回阶段','生成数','检查数','runId'];function draw(){let q=document.querySelector('#q').value.toLowerCase(),scope=document.querySelector('#scope').value,b=document.querySelector('#budget').value,ok=document.querySelector('#ok').value;shown=records.filter(r=>(!scope||r[0]===scope)&&(!b||r[4]===+b)&&(!ok||r[6]===+(ok==='yes'))&&(!q||JSON.stringify(r.slice(0,13)).toLowerCase().includes(q)));page=Math.min(page,Math.max(0,Math.ceil(shown.length/n)-1));document.querySelector('#count').textContent=`共 ${shown.length} 条；第 ${page+1}/${Math.max(1,Math.ceil(shown.length/n))} 页`;let body=document.querySelector('#calls tbody');body.replaceChildren();for(let r of shown.slice(page*n,(page+1)*n)){let tr=document.createElement('tr');for(let i=0;i<13;i++){let td=document.createElement('td');td.textContent=r[i];tr.append(td)}let td=document.createElement('td'),de=document.createElement('details'),su=document.createElement('summary'),pre=document.createElement('pre');su.textContent='原因/质量';pre.textContent=JSON.stringify({attemptReasons:r[13],rawMetrics:r[14]},null,2);de.append(su,pre);td.append(de);tr.append(td);body.append(tr)}}function move(k){page=Math.max(0,page+k);draw()}window.addEventListener('DOMContentLoaded',draw);'''
appendix='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>完整数据附录</title><style>body{font:14px/1.6 "Microsoft YaHei",sans-serif;margin:25px;color:#183443}table{border-collapse:collapse;min-width:900px;font-size:12px}td,th{padding:6px;border:1px solid #dce4ea}th{background:#e7f3f4;position:sticky;top:0}details{margin:14px 0}summary{cursor:pointer;font-weight:bold}input,select,button{padding:8px;margin:8px}pre{max-width:500px;white-space:pre-wrap}.scroll{overflow:auto;max-height:75vh}a{color:#087f8c}</style><h1>完整数据附录</h1><p><a href="报告.html">返回报告正文</a>｜30096次载荷调用保留全部成功和失败。原始JSON字段在data压缩文件中；下表仅分页，不删记录。输入任务ID、方法名或runId搜索。</p><input id="q" placeholder="例如 cylinder_02 / ddpm_basic" oninput="page=0;draw()"><select id="scope" onchange="page=0;draw()"><option value="">全部实验</option><option value="main">主实验</option><option value="controls">补充对照</option><option value="quality">质量策略</option></select><select id="budget" onchange="page=0;draw()"><option value="">全部预算</option><option>0.5</option><option>1</option><option>2</option><option>5</option><option>10</option></select><select id="ok" onchange="page=0;draw()"><option value="">成功和失败</option><option value="yes">成功</option><option value="no">失败</option></select><p id="count"></p><button onclick="move(-1)">上一页</button><button onclick="move(1)">下一页</button><div class="scroll"><table id="calls"><thead><tr>'''+''.join('<th>'+x+'</th>' for x in ['组','任务','分组','方法','预算秒','区组','最终合格','实际秒','终止原因','返回阶段','生成数','检查数','runId','详细'])+'''</tr></thead><tbody></tbody></table></div>'''
appendix+='<details><summary>全部方法 × 预算 × 泛化分组汇总</summary><div class="scroll">'+table(['组','方法','预算','分组','合格/调用','端点通过子集合格率','实际平均秒','实际P50','实际P95','成功阶段'],[[r['scope'],L.get(r['method'],r['method']),r['B'],r['group'],str(r['qualified'])+'/'+str(r['n'])+' '+pct(r['rate']),pct(r['rateEligible']) if r['rateEligible'] is not None else '-',f"{r['returnMean']:.4f}",f"{r['returnP50']:.4f}",f"{r['returnP95']:.4f}",html.escape(str(r['stageCounts']))] for r in S])+'</div></details>'
appendix+='<details><summary>全部配对差与95%区间</summary>'+table(['组','预算','比较','差/百分点','区间'],[[r['group'],r['B'],L[r['method']]+'−'+L[r['baseline']],f"{100*r['difference']:+.2f}",str([round(100*x,2) for x in r['ci95']])] for r in C])+'</details>'
appendix+='<details><summary>共同成功任务的全部质量配对</summary><pre>'+html.escape(json.dumps(Q,ensure_ascii=False,indent=2))+'</pre></details>'
appendix+='<details><summary>全部阶段真实耗时分解</summary><pre>'+html.escape(json.dumps([{'scope':r['scope'],'method':r['method'],'B':r['B'],'group':r['group'],'meanExclusiveSeconds':r['meanExclusiveSeconds']} for r in S],ensure_ascii=False,indent=2))+'</pre></details>'
appendix+='<p><a href="evidence/payload_budget10_failure_audit_v1__failure_audit.html">逐条失败报告</a>；其“尚未判定”段落以本报告合并补充对照后的结论为准。原始数据路径解释见报告第10页。编码、训练数据与几何定义见code目录。PDF仅正文，完整数据在此附录及data目录。</p>'
appendix+='<script>const records='+json.dumps(compact,ensure_ascii=False,separators=(',',':')).replace('</',r'<\/')+';'+script+'</script></html>'
(O/'附录.html').write_text(appendix,encoding='utf-8')
plain=re.sub('<[^>]+>','',doc);print('Chinese characters',len(re.findall('[\u4e00-\u9fff]',plain)),'page sections',len(pages),'appendix rows',len(compact))
shutil.copy2(__file__,O/'code'/pathlib.Path(__file__).name)
shutil.copy2(P/'build_complete_report_v1.py',O/'code'/'build_complete_report_v1.py')
