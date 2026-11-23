"""Recomputable diagnosis and matched development results, with failed trials."""
from pathlib import Path
import csv,hashlib,json,html
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[3]
ART=ROOT/'chapter5/artifacts/control_force_v62'
OUT=ART/'raw_v74'
def read(path):return json.loads(path.read_text(encoding='utf-8-sig'))
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main(version=74):
    global OUT
    OUT=ART/f'raw_v{version}'
    s=read(ART/'raw_v73/sensitivity.json');new=read(OUT/'raw_report.json');old=read(ART/'candidate_v64/candidate_report.json')
    rows=[]
    for name,rr in [('open_loop_temporal',old['rows']),('feedback_innovation',new['rows'])]:
        for r in rr:
            c=r.get('check',r.get('raw_check'))
            rows.append(dict(method=name,index=r.get('index',r.get('selection_index')),candidate=r['candidate'],mode=r['mode'],
                accepted=c['accepted'],internal_physics=c['internal_check']['accepted'],terminal_checked=c['terminal_checked'],
                terminal_ratio=c.get('terminal_region_ratio'),constraint_violation=c['inequality_violation'],
                force_error_N=r.get('force_error_rms_N',r.get('repaired_force_rms_error_N'))))
    with (OUT/'matched_candidates.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    candidate_summary=[]
    for method in ['open_loop_temporal','feedback_innovation']:
        rr=[x for x in rows if x['method']==method]
        candidate_summary.append(dict(method=method,count=len(rr),full_accepted=sum(x['accepted'] for x in rr),
            internal_accepted=sum(x['internal_physics'] for x in rr),median_force_error_N=float(np.median([x['force_error_N'] for x in rr if x['force_error_N'] is not None]))))
    perturb=[]
    for amp in [0,.01,.1,1]:
        for feedback in [False,True]:
            rr=[r for r in s['rows'] if r['amplitude_N']==amp and r['feedback']==feedback]
            perturb.append(dict(amplitude_N=amp,feedback=feedback,count=len(rr),accepted=sum(r['check']['accepted'] for r in rr),
                internal_accepted=sum(r['check']['internal_check']['accepted'] for r in rr),
                realized_force_error_median_N=float(np.median([r['force_error_rms_N'] for r in rr])),
                endpoint_position_median_mm=float(np.median([r['endpoint_position_error_mm'] for r in rr]))))
    runs=[]
    for name in [f'raw_closed_v{version}',f'full_v{version}']:
        file=ART/name/'summary.json'
        if not file.exists():continue
        data=read(file)
        for r in data['rows']:
            m=r['metrics'];runs.append(dict(experiment=name,task=r['task'],mode=r['mode'],requested_duration=r['duration'],
                actual_duration=m.get('simulated_end_seconds'),completed=data['completed'],simulation_error=r['simulation_error'],
                safe=m['all_constraints_valid'],arrival=m['arrival'],position_mm=m['final_position']*1000,
                fallback=m.get('fallback_count'),wall_seconds=r['wall_seconds'],refresh_median_ms=float(np.median(r['refresh_seconds'])*1000),
                raw_audited=r.get('raw_audited'),raw_accepted=r.get('raw_accepted')))
    manifest={}
    for pattern in ['chapter5/reduced/*V73.m','chapter5/reduced/*V74.m',f'chapter5/reduced/*V{version}.m',f'chapter5/reduced/force_v62/*v{version}.py']:
        for p in ROOT.glob(pattern):manifest[str(p.relative_to(ROOT))]=digest(p)
    for p in [OUT/'seed_5101/weights.mat',OUT/'seed_5101/model.pt',ART/'raw_v73/innovation_data.mat',OUT/'candidates_5101.mat']:
        manifest[str(p.relative_to(ROOT))]=digest(p)
    for p in [OUT/'geometry.mat',OUT/'raw_report.json',OUT/'residual_audit.json',OUT/'coverage_audit.json']:
        if p.exists():manifest[str(p.relative_to(ROOT))]=digest(p)
    mode_summary=[]
    for method in ['open_loop_temporal','feedback_innovation']:
        for mode in ['ideal-force','nonideal-force']:
            rr=[x for x in rows if x['method']==method and x['mode']==mode]
            mode_summary.append(dict(method=method,mode=mode,count=len(rr),internal_accepted=sum(x['internal_physics'] for x in rr),full_accepted=sum(x['accepted'] for x in rr)))
    coverage=read(ART/'raw_v75/coverage_audit.json') if (ART/'raw_v75/coverage_audit.json').exists() else None
    guard=read(ART/'raw_v75/workspace_guard.json') if (ART/'raw_v75/workspace_guard.json').exists() else None
    residual=read(OUT/'residual_audit.json') if (OUT/'residual_audit.json').exists() else None
    result=dict(version=version,candidate_summary=candidate_summary,mode_summary=mode_summary,perturbations=perturb,runs=runs,parity=new['parity'],coverage_audit=coverage,residual_audit=residual,workspace_guard=guard,
        same_noise=new['same_noise'],raw_no_projection=new['raw_no_projection'],metadata=read(OUT/'seed_5101/metadata.json'),
        maximum_label_roundtrip_N=read(ART/'raw_v73/innovation_data.json')['maximum_roundtrip_force_N'],
        open_loop_radius=[min(r['open_loop_radius'] for r in s['gain_rows']),max(r['open_loop_radius'] for r in s['gain_rows'])],
        feedback_radius=[min(r['feedback_radius'] for r in s['gain_rows']),max(r['feedback_radius'] for r in s['gain_rows'])],
        development_only=True,formal_runs=0,formal_required=1440,source_manifest=manifest)
    (OUT/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    fig,ax=plt.subplots(1,2,figsize=(10,3.6));amps=[.01,.1,1]
    for fb,label in [(False,'Open-loop force'),(True,'Feedback innovation')]:
        rr=[r for r in perturb if r['feedback']==fb and r['amplitude_N']>0]
        ax[0].plot(amps,[r['endpoint_position_median_mm'] for r in rr],'o-',label=label)
        ax[1].plot(amps,[r['accepted']/r['count'] for r in rr],'o-',label=label)
    for a in ax:a.set_xscale('log');a.grid(alpha=.2);a.set_xlabel('Coordinate perturbation RMS [N]')
    ax[0].set_yscale('log');ax[0].set_ylabel('Median endpoint position error [mm]');ax[1].set_ylabel('Full gate pass fraction');ax[1].legend()
    fig.tight_layout();fig.savefig(OUT/'sensitivity.png',dpi=170);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,3.4));x=np.arange(2);ax.bar(x-.18,[r['internal_accepted']/r['count'] for r in candidate_summary],.36,label='Internal physics');ax.bar(x+.18,[r['full_accepted']/r['count'] for r in candidate_summary],.36,label='Full gate')
    ax.set_xticks(x,['Old raw temporal','New raw innovation']);ax.set_ylim(0,1);ax.set_ylabel('Pass fraction (same 96 draws)');ax.legend();fig.tight_layout();fig.savefig(OUT/'raw_candidates.png',dpi=170);plt.close(fig)
    def table(items):
        if not items:return '<p>尚无结果。</p>'
        keys=list(items[0]);return '<table><tr>'+''.join('<th>'+html.escape(k)+'</th>' for k in keys)+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(str(r[k]))+'</td>' for k in keys)+'</tr>' for r in items)+'</table>'
    body='''<!doctype html><html lang="zh-CN"><meta charset="UTF-8"><title>原始扩散候选诊断 V74</title><style>body{font:16px/1.65 system-ui;max-width:1150px;margin:35px auto;padding:0 20px;color:#233}table{border-collapse:collapse;font-size:13px}th,td{padding:6px;border:1px solid #ccd}img{max-width:100%}code{background:#eef;padding:3px}</style><h1>原始扩散候选：原因、可逆重构与固定条件验证</h1>
    <p>本报告为开发实验，正式运行 0/1440。使用既有 1006 个训练窗口和 23 个合格验证窗口，保留原来的第 24 个拒绝标签验证条件，因此原始候选比较共 96 次。仅种子 5101，移动障碍训练覆盖仍为零。</p>
    <h2>根因与数学实现</h2><p>固定状态的增广模型有开环不稳定模态。学习的绝对力序列即使与标签相近，开环展开也会累积状态误差，随后违反加速度等约束。采用预稳定反馈的输入坐标：<code>Δu_j = v_j - L([x_j;u_{j-1}]-ξ)</code>，<code>x_{j+1}=f_d(x_j,u_{j-1}+Δu_j)</code>。L 包含执行器和上一指令状态，由当前位姿的局部 Riccati 设计得到；ξ 是当前位姿的稳态锚点，模型不生成独立参考轨迹。标签通过同一真实模型反解 v，未降低安全门槛。</p>
    <p>该表示是局部数值预稳定，不是全局稳定性或约束保证。扰动实验在两个不同控制坐标上施加相同数值 RMS，实际力误差也单独记录，不能把坐标扰动的比值解释为普遍鲁棒性界。</p>
    <p>同时修复终端训练导数：实际检查取辅助位姿 q_s=q_H，原代理将它固定。新代理对该关系求导，仍属局部近似；平方终端误差替代压缩大违反量的对数惩罚，增加 t=99 条件生成训练比例。完整非线性、内部积分、执行器瞬态与终端下降检查仍单独执行。</p>
    <h2>文献依据与边界</h2><p><a href="https://onlinelibrary.wiley.com/doi/10.1002/adc2.102">预稳定预测控制</a>为改善困难开环预测提供依据；Stewart 控制增量可逆解码是本项目的实现调整。<a href="https://tml.stanford.edu/PDP.github.io/">PDP</a>指出物理控制中模仿误差累积和纠偏状态覆盖问题，支持下一阶段采集扰动恢复教师，但本轮未将其方法或结果移植宣称为本系统保证。<a href="https://proceedings.mlr.press/v283/romer25a.html">DPCC</a>依赖模型约束处理，不意味着原始网络样本自动可行。</p>
    <h2>固定标签扰动</h2><img src="sensitivity.png">'''+table(perturb)+'''<h2>同条件原始候选</h2><p>新模型生成后直接解码，未经 QP、限幅或尾部修复。完整通过率与内部物理通过率分开报告；不依据此测试选择模型。</p><img src="raw_candidates.png">'''+table(candidate_summary)+'''<h2>闭环验证</h2>'''+table(runs)+'''<p>原始候选闭环允许原有安全 QP 回退，但绕过预测优化和终端修复。完整方法继续使用原有约束优化；两类结果应分开解释。失败、未到达和回退均保留。</p><h2>复算</h2><p><a href="summary.json">完整统计和源码哈希</a> · <a href="matched_candidates.csv">逐候选 CSV</a> · <a href="raw_report.json">全部原始检查</a> · <a href="seed_5101/metadata.json">训练身份与模型哈希</a></p></html>'''
    body=body.replace('V74',f'V{version}').replace('<h2>闭环验证</h2>',table(mode_summary)+'<h2>闭环验证</h2>')
    body=body.replace('<h2>复算</h2>','<p>上述完整控制实验每次刷新还审计四个原始候选；V71 历史计时关闭了此诊断，因此不能直接把耗时差异归因为新模型。所有耗时是本机离线同步求解，未证明 10 Hz 墙钟实时性。</p><h2>复算</h2>')
    if version==75:
        body=body.replace('<h2>同条件原始候选</h2>','<h2>完整采样训练修复</h2><p>V75 从 V74 检查点继续训练，对完整 20 步 DDIM 保留梯度；物理损失覆盖 37 个带符号的机械约束类别、24 个输入及输入增量类别和 8 个障碍类别。每一类别取标签轨迹中最不利的控制节点；内部积分与执行器瞬态仍由独立非线性检查决定，训练代理不能代替它们。验证目标为终端比例、最大物理违反和标签误差，未使用正式测试选择模型。</p><p><a href="../raw_v74/index.html">保留的 V74 前轮结果与失败记录</a></p><h2>同条件原始候选</h2>')
        if coverage:
            rr=coverage['rows'];total=sum(r['legacy_acceleration_rows'] for r in rr)
            body=body.replace('<h2>完整采样训练修复</h2>',f'<h2>训练物理代理覆盖审计</h2><p>固定 {len(rr)} 个合格验证标签，旧代理共选择 {11*len(rr)} 行，其中加速度行 {total} 条；其余类别见<a href="coverage_audit.json">原始类别审计</a>。这解释了旧物理代理低损失与真实加速度违规可以同时出现。</p><h2>完整采样训练修复</h2>')
        if residual:
            def scalar_max(value):return max(value) if isinstance(value,list) else value
            kept=[dict(index=r['index'],candidate=r['candidate'],mode=r['mode'],label_accepted=r['label_accepted'],initial_acceleration=r['initial_maximum_acceleration_mps2'],
                initial_hard_violation=scalar_max(r['initial_hard_acceleration_violation']),physical_passed=r['physical_passed'],terminal_ratio=r['raw_check']['terminal_region_ratio'],brake_accepted=r['brake_check']['accepted']) for r in residual['rows']]
            body=body.replace('<h2>闭环验证</h2>','<h2>全部剩余拒绝候选</h2>'+table(kept)+'<p>9 个拒绝样本包括 5 个终端区域拒绝和 4 个晚段加速度违规。4 个加速度样本来自同一个旧标签未通过的条件，初态本身没有加速度违规，且该条件的制动初始化能通过完整检查，因此不能归因为不可行初态。拒绝条件均保留在 96 个样本的分母中；后续须改善动作尾段和教师标签覆盖，不能通过降低门槛消除失败。</p><h2>闭环验证</h2>')
    (OUT/'index.html').write_text(body,encoding='utf-8');print(json.dumps(candidate_summary));print('Report',OUT/'index.html')
if __name__=='__main__':main()
