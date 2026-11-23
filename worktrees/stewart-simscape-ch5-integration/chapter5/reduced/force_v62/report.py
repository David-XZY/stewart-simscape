"""Recompute development summaries and HTML from preserved raw artifacts."""
from pathlib import Path
import hashlib,html,json,subprocess
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3]
CH5=ROOT/'chapter5';OUT=CH5/'artifacts/control_force_v62'
OLD=CH5/'artifacts/control_force_v57/pilot'
def read(p):
    return json.loads(Path(p).read_text(encoding='utf-8-sig')) if Path(p).exists() else None
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def percentiles(a):
    a=[float(x) for x in a if x is not None and np.isfinite(x)]
    return dict(median=float(np.median(a)),p95=float(np.percentile(a,95)),maximum=float(max(a)),n=len(a)) if a else None
def summarize_candidates(path,candidates):
    report=read(path)
    if not report or not report['completed']:return None
    labels=sio.loadmat(OLD/'repaired.mat',simplify_cells=True,variable_names=['residual','accepted'])
    data=sio.loadmat(candidates,simplify_cells=True);ids=np.asarray(data['selection_index']).reshape(-1).astype(int)-1
    keep=np.asarray(labels['accepted']).reshape(-1).astype(bool)[ids]
    error=(data['residual'].T-labels['residual'].T[ids])[keep].reshape(-1,60,6)
    increments=np.diff(error,axis=1,prepend=np.zeros((len(error),1,6)))
    rr=report['rows']
    return dict(candidates=len(rr),label_comparable=int(keep.sum()),raw_accepted=sum(r['raw_check']['accepted'] for r in rr),
        projected_accepted=sum(r['projected_check']['accepted'] for r in rr),
        internal_physical_accepted=sum(r['raw_check']['internal_check']['accepted'] for r in rr),
        rate_rejected=sum(r['raw_check']['command_rate_violation']>1e-6 for r in rr),
        force_error_N=percentiles(np.sqrt(np.mean(error**2,axis=(1,2)))),
        increment_error_N=percentiles(np.sqrt(np.mean(increments**2,axis=(1,2)))),parity=report['parity'])
def summarize_runs(path):
    d=read(path)
    if not d:return dict(completed=False,rows=[],pending=True)
    rr=[]
    for r in d['rows']:
        m=r['metrics'];a=dict(task=r['task'],mode=r['mode'],duration=r.get('duration',15),safe_success=m['safe_success'],
            all_constraints_valid=m['all_constraints_valid'],arrival=m['arrival'],completion_seconds=m['completion_seconds'],
            final_position_mm=m['final_position']*1000,final_rotation_degrees=m['final_rotation']*180/np.pi,
            fallback_count=m['fallback_count'],solver_failure_count=m['solver_failure_count'],wall_seconds=r['wall_seconds'],
            command_rate_rms=m['command_rate_rms'],actual_force_rms=m['actual_force_rms'],violations=m['violations'])
        if 'refresh_seconds' in r:
            a.update(refresh_seconds=percentiles(r['refresh_seconds']),execution_seconds=percentiles(r['execution_seconds']),
                     refresh_over_100ms=sum(x>.1 for x in r['refresh_seconds']),execution_over_20ms=sum(x>.02 for x in r['execution_seconds']))
        rr.append(a)
    return dict(completed=d['completed'],error=d.get('error',''),rows=rr)
def table(headers,rows):
    return '<table><thead><tr>'+''.join('<th>'+html.escape(str(x))+'</th>' for x in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in r)+'</tr>' for r in rows)+'</tbody></table>'
def main():
    frozen={}
    files=[p for p in CH5.glob('*') if p.is_file() and p.suffix in ['.m','.py','.json']]
    guard=hashlib.sha256()
    for p in sorted(files):guard.update(p.name.encode());guard.update(p.read_bytes())
    root_guard=guard.hexdigest()
    for p in list((CH5/'reduced/force_v62').glob('*.py'))+list((CH5/'reduced').glob('*V6[2345]*.m')):
        frozen[str(p.relative_to(ROOT))]=sha(p)
    versions={'original_mlp':summarize_candidates(OLD/'candidate_v60/candidate_report.json',OLD/'candidates_5101.mat'),
              'temporal':summarize_candidates(OUT/'candidate_v64/candidate_report.json',OUT/'temporal_v64/candidates_5101.mat')}
    teacher=read(OUT/'teacher_audit.json');interval=read(OUT/'teacher_interval_audit.json');labels=read(OUT/'labels_v64/label_audit.json')
    select=read(OUT/'labels_v65/label_selection.json');raw=read(OUT/'raw_audit_v65.json')
    runs={v:summarize_runs(p) for v,p in [('v60_full',OLD/'closed_loop_v60/summary.json'),('v63_short',OUT/'short_v63/summary.json'),
        ('v64_short',OUT/'short_v64/summary.json'),('v64_full',OUT/'full_v64/summary.json'),('teacher_v65',OUT/'teacher_v65/summary.json')]}
    status=read(CH5/'artifacts/performance_probe/worker_status.json')
    diff=subprocess.check_output(['git','diff','--name-only','HEAD'],cwd=ROOT,text=True).strip()
    summary=dict(formal=False,formal_ready=False,formal_runs=0,formal_required=1440,versions=versions,runs=runs,
        teacher_source_audit=teacher,teacher_interval_counts={m:dict(n=sum(r['mode']==m for r in interval['rows']),
            before=sum(r['before']['accepted'] for r in interval['rows'] if r['mode']==m),after=sum(r['after']['accepted'] for r in interval['rows'] if r['mode']==m))
            for m in ['ideal-force','nonideal-force']} if interval else None,
        label_counts=dict(old=sum(r['previous_label_accepted'] for r in labels['rows']),distributed=sum(r['check']['accepted'] for r in labels['rows']),
            selected=select['accepted'] if select else None,n=len(labels['rows'])) if labels else None,
        inference=read(OUT/'inference_v65.json'),worker=status,restoration=read(OUT/'finalized_v65.json'),
        baseline_guard=root_guard,baseline_guard_matches=root_guard=='a0735e0e37317afcf4107359797e9fbab71fcb886584beac318b42edbcb050d6',
        tracked_changes=diff,git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        data_boundary='immutable successful V57 teachers; 1006 train/23 validation windows; moving-obstacle coverage zero; seed 5101 development only',
        source_sha256=frozen)
    if raw:
        summary['raw_failure_components']={model:{f:dict(rejected=sum(r[f]>1e-6 for r in raw['rows'] if r['model']==model),
             value=percentiles(r[f] for r in raw['rows'] if r['model']==model)) for f in ['acceleration_violation_mps2','length_violation_m',
             'speed_violation_mps','singularity_violation','command_force_violation_N','command_rate_violation_Nps']}
             for model in ['mlp_v60','temporal_v64']}
    figures=[]
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'white'})
    if all(versions.values()):
        fig,axes=plt.subplots(1,2,figsize=(9,3.5));names=['MLP V60','Temporal V64']
        for ax,f,label in zip(axes,['force_error_N','increment_error_N'],['Force-sequence error (N)','Increment error (N)']):
            ax.bar(names,[versions[v][f]['median'] for v in versions],color=['#8797a5','#177f83']);ax.set_ylabel('Median '+label)
        fig.tight_layout();fig.savefig(OUT/'candidate_errors.png',dpi=160);plt.close(fig);figures.append(('candidate_errors.png','固定 92 个可比候选，对完整通过检查的修正标签计算误差；开发窗口不作为正式独立任务。'))
    if runs['v64_full']['rows']:
        fig,ax=plt.subplots(figsize=(8,3.6))
        for version,directory in [('V60',OLD/'closed_loop_v60'),('V64',OUT/'full_v64')]:
            for task in [1,4]:
                row=read(directory/f'{task:03d}_nonideal_force.json')
                if not row:continue
                a=np.sort(np.asarray(row['refresh_seconds'])*1000);ax.plot(a,np.arange(1,len(a)+1)/len(a),label=f'{version} task {task}')
        ax.axvline(100,color='#aa3333',ls='--',label='100 ms');ax.set_xlabel('Complete planning time (ms)');ax.set_ylabel('Empirical CDF');ax.legend(fontsize=9);fig.tight_layout();fig.savefig(OUT/'planning_cdf.png',dpi=160);plt.close(fig)
        figures.append(('planning_cdf.png','相同 15 s 开发任务的完整规划耗时，包含生成、优化和物理检查；仿真时间不能解释为墙钟实时性。'))
    rows=[]
    for name,v in versions.items():
        if v:rows.append([name,f"{v['raw_accepted']}/{v['candidates']}",f"{v['projected_accepted']}/{v['candidates']}",f"{v['force_error_N']['median']:.3f}",f"{v['increment_error_N']['median']:.3f}"])
    runrows=[]
    for version in ['v63_short','v64_short','v64_full','teacher_v65']:
        for r in runs[version]['rows']:
            runrows.append([version,r['task'],r['mode'],r['duration'],r['all_constraints_valid'],r['arrival'],f"{r['final_position_mm']:.3f}",
                f"{r['refresh_seconds']['median']*1000:.1f}" if 'refresh_seconds' in r else '—',r['fallback_count'],f"{r['wall_seconds']:.1f}"])
    failures=[p.name for p in OUT.glob('*failed*status.json')]
    (OUT/'summary_v65.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    (OUT/'source_manifest_v65.json').write_text(json.dumps(frozen,indent=2),encoding='utf-8')
    sections=["<h1>第五章开发验证：V62–V65</h1><p class='notice'>开发证据，formal_ready=false，正式实验 0 / 1440。仅训练种子 5101；不得据此宣称五种子训练、正式对比或稳定性证明完成。</p>",
        "<h2>实现范围</h2><p>保持控制决策与动力学展开接口。复用已有 MATLAB 会话；新增 QP 分解复用、终端精确记忆、重复检查消除、独立数值热启动、0.1 s 失败重试节奏及候选审计预算。任何执行计划须通过完整内部网格与执行器瞬态检查；安全 QP 失败或保持旧输入不视为安全保证。</p>",
        "<p>时序卷积与 FiLM 条件化借鉴 <a href='https://arxiv.org/html/2303.04137v5'>Diffusion Policy</a>；物理投影思想参考 <a href='https://proceedings.mlr.press/v283/romer25a.html'>DPCC</a>；部分敏感度更新属于本项目工程近似，参考 <a href='https://docs.acados.org/features/index.html'>acados RTI</a> 的结构思想，代码未使用 acados，也不借用其性能或理论结论。</p>",
        "<h2>模型和固定候选</h2><p>新模型使用五个膨胀时序卷积块、73,222 个参数、完整 20 步 DDIM；1006 个训练窗口与 23 个验证窗口，按完整采样误差与局部物理代理选择检查点。终端代理为线性化动力学的二次范数；碰撞训练行只覆盖选定节点，均不能代替完整物理检查。当前移动障碍有效教师覆盖仍为零。</p>",
        table(['版本','原始通过','优化通过','力序列误差中位数 N','增量误差中位数 N'],rows),
        "<p>误差改善与可行性改善分别报告。通过数没有增加即不得声称已解决原始候选失败。训练与推理固定噪声、标签及来源信息均有原始文件和哈希。</p>",
        "<h2>Simscape 结果</h2>",table(['版本','任务','执行器','时长 s','全约束有效','到达','末位置 mm','规划中位数 ms','回退','墙钟 s'],runrows),
        "<p>3 s 移动障碍运行仅检查初段安全；目标暂时被阻挡，不以短时到达作为通过条件。正式任务上限仍为 15 s，不依据开发结果修改测试期限。</p>",
        "<h2>教师与标签</h2><p>100 个原失败移动障碍教师均保留且未纳入训练。审计将模型力计算的加速度与速度数值导数分开，证实大量越界发生于采样点之间。48 个固定状态加入保持输入区间约束后，接受数从 24 增至 31；起始状态已不可行时不标记为修复成功。</p>",
        "<p>72 个预先固定窗口比较原尾段修正与全时域平滑修正；分别都接受 70 个。新修正降低尾部异常的 P95，但中位数稍增。V65 对两个修正初值完整重审，并按力/20 N 与增量/2 N 的平方误差之和择优。此选择结果尚未用于上述时序模型训练。</p>",
        "<h2>复算与保存</h2><p>摘要：<a href='summary_v65.json'>summary_v65.json</a>；源码哈希：<a href='source_manifest_v65.json'>source_manifest_v65.json</a>；候选原始报告：<a href='candidate_v64/candidate_report.json'>candidate_report.json</a>；教师源审计：<a href='teacher_audit.json'>teacher_audit.json</a>。</p>",
        '<p>生产源码指纹校验：'+str(summary['baseline_guard_matches'])+'；跟踪文件改动：'+html.escape(diff or '无')+'。实现/设置失败状态均保留：'+html.escape(', '.join(failures))+'.</p>',
        "<pre>chapter5/.venv/Scripts/python.exe chapter5/reduced/force_v62/report.py</pre>"]
    for f,c in figures:sections.append(f"<figure><img src='{f}' alt='{html.escape(c)}'><figcaption>{html.escape(c)}</figcaption></figure>")
    page="<!doctype html><html lang='zh-CN'><meta charset='utf-8'><title>Chapter 5 development V62–V65</title><style>body{font:16px/1.6 'Microsoft YaHei',sans-serif;max-width:1120px;margin:32px auto;padding:0 20px;color:#23313b}h1{font-size:28px}h2{font-size:22px;margin-top:30px}.notice{background:#fff3db;border-left:4px solid #c18b23;padding:12px}table{border-collapse:collapse;width:100%;font-size:14px;margin:16px 0}th,td{text-align:left;border-bottom:1px solid #d4dde3;padding:9px}th{background:#eaf0f3}img{max-width:100%;height:auto}figcaption{color:#596b76;font-size:14px}pre{overflow:auto;background:#edf2f4;padding:14px}a{color:#17657b}</style><body>"+''.join(sections)+"</body></html>"
    (OUT/'index_v65.html').write_text(page,encoding='utf-8');print('Report saved:',OUT/'index_v65.html');print('Production guard matches:',summary['baseline_guard_matches'])
if __name__=='__main__':main()
