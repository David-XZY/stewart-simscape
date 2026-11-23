"""Build the reader-facing chart atlas; leave numerical runs untouched."""
import pathlib,json,html,gzip,shutil,argparse
P=pathlib.Path(__file__).resolve().parent
a=argparse.ArgumentParser();a.add_argument('--output',type=pathlib.Path,default=P/'payload_endpoint_pass_figures_v1_20260927');O=a.parse_args().output
def read(n):return json.loads((O/'data'/n).read_text(encoding='utf-8'))
config=json.loads((O/'analysis_config.json').read_text(encoding='utf-8'));M=config['methods'];L=config['labels'];S=read('success_summary.json');C=read('paired_success.json');J=read('common_success.json');Q=read('quality_summary.json')
def table(headers,rows):return '<table><thead><tr>'+''.join('<th>'+str(s)+'</th>' for s in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+str(s)+'</td>' for s in row)+'</tr>' for row in rows)+'</tbody></table>'
def stat(m,b):return next(s for s in S if s['method']==m and s['B']==b)
def cell(m,b):
 s=stat(m,b);return '未测' if s['status']=='unmeasured' else f"{s['qualified']}/438 · {100*s['mean']:.2f}%"
def contrast(a,b,B):
 s=next(c for c in C if (c['method'],c['baseline'],c['B'])==(a,b,B));return f"{100*s['mean']:+.2f} [{100*s['ci95'][0]:+.2f}, {100*s['ci95'][1]:+.2f}]"
figures=[
 ('01_success_budget','预算与合格返回率','landscape',f"10秒：基础扩散对普通回归的成功率差为{contrast('ddpm_basic','regression',10)}个百分点；签名扩散对签名回归为{contrast('ddpm_signature','regression_signature',10)}。签名扩散对基础扩散为{contrast('ddpm_signature','ddpm_basic',10)}，区间跨0，不能确认签名在全部任务上稳定增益。",'分母：每已测点146任务×3区组=438次。按20场景成簇、共享种子区组重抽2000次；零成功的经验bootstrap区间会退化为0，不等于真实成功概率被证明为0。缺测不连线。'),
 ('02_generalization','泛化与载荷适用范围','landscape','同一个总成功率可能对应很不同的任务分布。图中分别拆开泛化类型、形状和环境，不把小组均值直接当整个方法优劣；分组点估计及区间均保留。','G0/G1/G2/G3保留39/40/39/28任务。各形状/环境分母见图。每次重复不增加独立任务数；小组区间更不稳定。'),
 ('03_time_decomposition','真实耗时与时间分解','portrait','大部分学习方法调用很快结束，但少量失败尝试耗尽预算，因此平均耗时与中位数差别明显。IHSID在1秒、2秒没有合格返回，5秒、10秒有成功；必须同时看返回率，不能只比较成功样本的速度。','全部438次调用计时，失败未删除、超时未截断。分项闭合。4次求解异常路径只有复合计时，归入NLP并保留原始字段；虚线为预算。'),
 ('04_outcomes_failures','成功来源与失败去向','landscape','上排结果分类互斥，总和严格等于438；下排是失败调用中曾出现的拒绝原因，同次调用多个候选可触发不同原因，因此不能把下排各列相加当失败总数。','直接成功、精修成功、回退成功分别计数。间隙未获证书不等于碰撞；腿运动码尚未区分腿长、速度、加速度；预算终止不等于任务无解。'),
 ('05_quality_matched','共同成功轨迹的品质','landscape','在同一匹配集合上，平台线jerk RMS中位数依次为1.605、0.750、1.856、0.759、0.166 m/s³（按图中方法顺序）。扩散的可行返回率收益没有自动转化成更平滑的轨迹；应结合全部12项原始指标，而非用总分掩盖代价。','B=10秒；248组匹配调用来自83任务、15场景。仅描述共同成功子集。箱体为25%–75%分位、横线为中位数、须为1.5倍IQR；散点保留全部观测。机械功不是电能。'),
 ('06_safety_margins','约束余量与稳健程度','landscape','全部点已经通过物理验收，但余量仍能反映约束有多紧。更接近阈值不等于必然不安全，也不应把最大利用率越大当越好。','与图5完全相同的248组匹配调用。间隙只画存在障碍的适用记录；无障碍值标为不适用，不填大数。使用已保存的在线完整报告和统一质量指标；离线复查通过标记已核验，不冒称另有未保存的加密极值。'),
 ('07_candidates','候选数量与生成效率','portrait','回归每次最多一个候选；扩散可继续生成候选。有些调用很早成功，有些生成许多仍需精修或失败。这说明应同时看唯一生成数、阶段时间和最终返回，不能用候选多来证明有效多样性。','四种学习方法。左图使用非线性刻度容纳0–64；气泡面积与调用数正相关。候选阶段包含输入、编码、生成、粗排及进入公共后处理前的检查；不把重复验收计作新候选，也不将关联解读成因果。')]
head='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>五方法端点通过任务集结果图册</title><style>
body{font:16px/1.75 "Microsoft YaHei",sans-serif;background:#edf2f5;color:#173143;margin:0}section{background:white;max-width:1450px;margin:25px auto;padding:30px 40px}h1{font-size:26px;line-height:1.35;color:#0b6975}h2{font-size:21px}p{margin:10px 0}.note{font-size:13px;color:#536a79}.chart{width:100%;height:auto}table{border-collapse:collapse;width:100%;font-size:14px}td,th{padding:9px;border-bottom:1px solid #d7e0e6;text-align:left}th{background:#e7f3f5}a{color:#087a89}nav{margin:12px 0}nav a{margin-right:18px}input,select,button{padding:8px;margin:6px}details{margin:18px 0}summary{cursor:pointer;font-weight:bold}.scroll{overflow:auto;max-height:75vh}pre{white-space:pre-wrap;font-size:12px}.callout{padding:16px;border-left:4px solid #0e8991;background:#eef7f7}.footer{font-size:11px;color:#6b7c86}
@page landscape{size:A3 landscape;margin:12mm}@page portrait{size:A3 portrait;margin:12mm}
@media print{body{background:white;font-size:12px;line-height:1.55}section{max-width:none;margin:0;padding:0;break-after:page}section:last-child{break-after:auto}.landscape{page:landscape;height:273mm}.portrait{page:portrait;height:396mm}h1{font-size:20px;margin:0 0 4mm}.chart{width:100%;object-fit:contain}.landscape .chart{height:227mm}.portrait .chart{height:348mm}.caption{font-size:11px;margin:3mm 0}.note{font-size:10px;margin:2mm 0}nav,.screenonly{display:none}.cover h1{font-size:29px}.cover{font-size:16px}table{font-size:14px}.footer{font-size:9px}}
</style>'''
cover='<section class="landscape cover"><h1>五方法结果图册<br>端点通过任务集 · 1 / 2 / 5 / 10秒</h1><p>首轮已有数据；原正式版本的补充分析，不混入新训练版本。</p><nav><a href="五方法结果图册.pdf">PDF图册</a><a href="数据附录.html">可筛选调用与统计附录</a></nav><div class="callout">固定剔除46个端点拒绝任务：32个腿长越界、14个间隙未获证书。每个已测方法/预算原576次减138次，保留146任务、20场景、438次调用。14个间隙拒绝任务不能全部称为已证明不可行。</div><h2>最终物理合格返回</h2>'+table(['方法','1秒','2秒','5秒','10秒'],[[L[m]]+[cell(m,b) for b in [1,2,5,10]] for m in M])+'<h2>阅读说明</h2><p>成功统一使用最终复查后的合格标记；其余超时、候选违规和复查失败不删。图5、6采用五方法共同成功的248组匹配调用：83任务、15场景。置信区间采用任务等权的重抽样总数比，保持场景任务数量差异和跨场景共享种子。</p><p>七张图依次回答：成功率、泛化、耗时、成功来源/失败原因、轨迹品质、约束余量、候选效率。可行率、品质和速度分别报告，不使用一个综合分替代全部判断。</p><p class="note">首轮未启动补测。签名回归1秒、5秒缺测，不插值；后续补876次独立调用并核对原环境后，以新版本更新图册。当前所有主图只包含本次指定五方法。</p><div class="footer">数据快照：2026-09-27｜PNG 260 dpi、可编辑文字SVG、完整过滤日志、统计数据和复现脚本随包。</div></section>'
sections=[]
for i,(name,title,orientation,conclusion,note) in enumerate(figures,1):
 sections.append(f'<section class="{orientation}"><h1>图{i}　{title}</h1><nav><a href="figures/{name}.png">PNG</a><a href="figures/{name}.svg">SVG</a><a href="数据附录.html">数据</a></nav><img class="chart" src="figures/{name}.svg" alt="{title}"><p class="caption"><b>如何理解：</b>{conclusion}</p><p class="note">{note}</p><div class="footer">端点通过子集补充分析｜首轮缺测保留｜图{i}/7</div></section>')
(O/'index.html').write_text(head+cover+''.join(sections)+'</html>',encoding='utf-8')
calls=[]
with gzip.open(O/'data/filtered_calls.jsonl.gz','rt',encoding='utf-8') as f:
 for line in f:
  r=json.loads(line);calls.append([r['taskId'],r['baseSceneId'],r['method'],r['B'],r['repeat'],int(r['finalQualified']),r['actualReturnTime'],r['returnStage'],r['reason'],r['candidateGenerated'],r['candidateChecked'],r['runId']])
fields=['任务','场景','方法','预算秒','区组','最终合格','实际总秒','返回阶段','终止原因','唯一生成数','检查次数','runId']
appendix=head+'<section><h1>数据附录</h1><a href="index.html">返回图册</a><p>共7884条已测调用。方法和预算过滤后没有记录，表示未测，不表示零成功。原始字段与质量指标见data/filtered_calls.jsonl.gz；共同成功的原始绘图值见matched_observations.json。</p><input id="q" placeholder="任务ID或runId" oninput="page=0;draw()"><select id="m" onchange="page=0;draw()"><option value="">全部方法</option>'+''.join(f'<option value="{m}">{L[m]}</option>' for m in M)+'</select><select id="b" onchange="page=0;draw()"><option value="">全部预算</option>'+''.join(f'<option>{b}</option>' for b in [1,2,5,10])+'</select><p id="count"></p><button onclick="step(-1)">上一页</button><button onclick="step(1)">下一页</button><div class="scroll"><table id="calls"><thead><tr>'+''.join('<th>'+x+'</th>' for x in fields)+'</tr></thead><tbody></tbody></table></div></section>'
for file,title in [('excluded_tasks.json','排除清单（46个任务）'),('success_summary.json','成功率与95%区间'),('paired_success.json','配对成功率差'),('strata.json','泛化/形状/环境全部分组'),('timing.json','互斥耗时分解'),('outcomes.json','互斥成功/失败分类'),('failure_causes.json','可重叠的候选拒绝原因'),('quality_summary.json','共同成功品质统计'),('safety_summary.json','共同成功约束余量'),('pairwise_quality.json','两两共同成功的品质配对差'),('candidate_summary.json','候选数量与时间'),('integrity.json','完整性检查')]:
 appendix+='<section><details><summary>'+title+'</summary><pre>'+html.escape(json.dumps(read(file),ensure_ascii=False,indent=2))+'</pre></details></section>'
appendix+='''<script>let page=0;const n=60;function draw(){const q=document.querySelector('#q').value.toLowerCase(),m=document.querySelector('#m').value,b=document.querySelector('#b').value;const rs=rows.filter(r=>(!m||r[2]===m)&&(!b||r[3]===+b)&&(!q||JSON.stringify(r).toLowerCase().includes(q)));page=Math.min(page,Math.max(0,Math.ceil(rs.length/n)-1));document.querySelector('#count').textContent=`${rs.length} 条；第 ${page+1}/${Math.max(1,Math.ceil(rs.length/n))} 页`;let body=document.querySelector('#calls tbody');body.replaceChildren();for(let r of rs.slice(page*n,(page+1)*n)){let tr=document.createElement('tr');r.forEach((v,i)=>{let td=document.createElement('td');td.textContent=i===2?labels[v]:i===6?v.toFixed(6):v;tr.append(td)});body.append(tr)}}function step(k){page=Math.max(0,page+k);draw()}window.addEventListener('DOMContentLoaded',draw);const labels='''+json.dumps(L,ensure_ascii=False)+';const rows='+json.dumps(calls,ensure_ascii=False,separators=(',',':')).replace('</',r'<\/')+';</script></html>'
appendix=appendix.replace('共7884条已测调用',f'共{len(calls)}条已测调用')
if not config['missingCells']:
 page=(O/'index.html').read_text(encoding='utf-8').replace('首轮已有数据','原有数据及独立补测').replace('首轮未启动补测。签名回归1秒、5秒缺测，不插值；后续补876次独立调用并核对原环境后，以新版本更新图册。','签名回归1秒、5秒已加入独立补测；来源和兼容性记录随包，原首轮保留。').replace('首轮缺测保留','补测版本')
 (O/'index.html').write_text(page,encoding='utf-8')
(O/'数据附录.html').write_text(appendix,encoding='utf-8')
supplement='''首轮不启动实验。原签名回归1秒、5秒为空缺，共需876次真实调用。
补测输出独立目录，calls.jsonl.gz仅含146任务×3原区组×2预算。
补测前核对原数值代码、模型、数据、硬件、运行库、线程、缓存规则；不匹配则不能拼接，改为受影响预算五方法同批重跑。
仅在核验通过后写compatibility.json：codeHash、libraryHash取本包analysis_config.json；originalHardwareRuntimeThreadCacheIdentityVerified=true必须有实际核对证据。
执行：python plot_endpoint_five_methods.py --source <原报告data目录> --supplement <补测目录> --output <全新v2目录>
再执行：python deliver_endpoint_five_methods.py --output <全新v2目录>
原v1目录与图表保留。该接口只接收补测，不执行规划器；不要把真实失败填成未测。
全五方法重跑时，应从新批次重新建立统一来源，不把该补测入口用于混合不兼容运行。
'''
(O/'补测与复现.txt').write_text(supplement,encoding='utf-8')
shutil.copy2(__file__,O/pathlib.Path(__file__).name)
print(json.dumps({'atlasFigures':len(figures),'retainedCalls':len(calls),'commonTasks':J['tasks']},ensure_ascii=False))
