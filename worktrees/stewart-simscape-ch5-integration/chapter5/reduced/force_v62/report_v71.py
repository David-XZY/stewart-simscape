"""Recompute V62--V71 development evidence; never overwrite earlier reports."""
from pathlib import Path
import csv, hashlib, html, json, subprocess
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from report import ROOT, CH5, OUT, OLD, read, sha, percentiles, summarize_candidates, table

VERSIONS = {
    'V60': OLD/'closed_loop_v60',
    'V63 short': OUT/'short_v63', 'V64 short': OUT/'short_v64',
    'V64': OUT/'full_v64', 'V68 short': OUT/'short_v68',
    'V68 fault': OUT/'fault_v68', 'V68': OUT/'full_v68',
    'Baseline V65': OUT/'teacher_v65',
    'Baseline V67 short': OUT/'teacher_short_v67', 'Baseline V67': OUT/'teacher_full_v67',
    'V70 short': OUT/'terminal_short_v70', 'V70': OUT/'terminal_full_v70',
    'V71 fault': OUT/'fault_v71', 'V71': OUT/'full_v71',
}

def run_summary(directory):
    d=read(directory/'summary.json')
    if not d: return {'completed':False,'pending':True,'rows':[]}
    rows=[]
    for r in d['rows']:
        m=r['metrics']; requested=r.get('duration',15)
        actual=m.get('simulated_end_seconds')
        complete=actual is not None and actual>=requested-1e-8 and not r.get('simulation_error','')
        z={'task':r['task'],'mode':r['mode'],'requested_seconds':requested,
           'simulated_end_seconds':actual,'episode_complete':complete,
           'all_constraints_valid':m['all_constraints_valid'],'arrival':m['arrival'],
           'tight_arrival':m.get('tight_arrival'),'safe_success':bool(m['safe_success'] and complete),
           'completion_seconds':m['completion_seconds'],'final_position_mm':1000*m['final_position'],
           'final_rotation_degrees':np.rad2deg(m['final_rotation']),
           'wall_seconds':r['wall_seconds'],'fallback_count':m['fallback_count'],
           'solver_failure_count':m['solver_failure_count'],
           'predictive_solver_failures':m.get('predictive_solver_failures'),
           'predictive_solves':m.get('predictive_solves'),
           'command_rate_rms':m['command_rate_rms'],'actual_force_rms':m['actual_force_rms'],
           'violations':m['violations'],'simulation_error':r.get('simulation_error',''),
           'admitted':bool(r.get('admitted',False) and complete),
           'planning_seconds':m.get('replanning_seconds'),'initial_planning_seconds':m.get('initial_planning_seconds')}
        for key in ['refresh_seconds','execution_seconds']:
            if key in r:z[key]=percentiles(r[key])
        if 'refresh_seconds' in r:
            z['refresh_count']=len(r['refresh_seconds']);z['refresh_over_100ms']=sum(t>.1 for t in r['refresh_seconds'])
        stem=f"{int(r['task']):03d}" if isinstance(r['task'],(int,float)) else str(r['task'])
        mat=directory/f"{stem}_{r['mode'].replace('-','_')}.mat"
        if mat.exists():
            data=sio.loadmat(mat,simplify_cells=True,variable_names=['predictions','records'])
            predictions=data.get('predictions',[]);predictions=predictions if isinstance(predictions,list) else [predictions]
            terminal=[float(p['time']) for p in predictions if isinstance(p,dict)
                      and isinstance(p.get('prediction'),dict) and p['prediction'].get('terminal_active',False)]
            z['terminal_samples']=len(terminal);z['terminal_first_seconds']=terminal[0] if terminal else None
            z['terminal_last_seconds']=terminal[-1] if terminal else None
            z['terminal_samples_after_switch']=sum(t>=8 for t in terminal) if r['task']==3 else None
            records=data.get('records',[]);records=records if isinstance(records,list) else [records]
            fallback=[float(p['time']) for p in records if p['diagnostic'].get('usedFallback',False)]
            z['first_fallback_seconds']=fallback[0] if fallback else None
            z['last_fallback_seconds']=fallback[-1] if fallback else None
            z['raw_file']=str(mat.relative_to(OUT)) if mat.is_relative_to(OUT) else str(mat)
        rows.append(z)
    return {'completed':d['completed'],'pending':False,'error':d.get('error',''),'rows':rows}

def recovery_summary():
    d=read(OUT/'recovery_audit_v71.json')
    if not d:return None
    return {'completed':d['completed'],'rows':[{k:r[k] for k in
        ['source','task','time','reset_to_current_brake','proposal_accepted','optimization_seconds']}
        |{'initializer_accepted':r['initializer_check']['accepted'],
          'qp_success':sum(s.get('solver_success',False) for s in r['solves'])} for r in d['rows']]}

def main():
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'white'})
    candidates={'MLP V60':summarize_candidates(OLD/'candidate_v60/candidate_report.json',OLD/'candidates_5101.mat'),
                'Temporal V64':summarize_candidates(OUT/'candidate_v64/candidate_report.json',OUT/'temporal_v64/candidates_5101.mat')}
    runs={v:run_summary(p) for v,p in VERSIONS.items()}
    raw=read(OUT/'raw_audit_v65.json');raw_counts={}
    if raw:
        for model in ['mlp_v60','temporal_v64']:
            rows=[r for r in raw['rows'] if r['model']==model]
            raw_counts[model]={key:{'rejected':sum(r[key]>1e-6 for r in rows),'values':percentiles(r[key] for r in rows)}
                for key in ['acceleration_violation_mps2','length_violation_m','speed_violation_mps',
                            'singularity_violation','command_force_violation_N','command_rate_violation_Nps']}
    sources=list((CH5/'reduced/force_v62').glob('*.py'))+[p for p in (CH5/'reduced').glob('*.m')
        if any(f'V{v}' in p.stem for v in range(62,72))]
    manifest={str(p.relative_to(ROOT)):sha(p) for p in sorted(sources)}
    guard=hashlib.sha256()
    for p in sorted(p for p in CH5.iterdir() if p.is_file() and p.suffix in ['.m','.py','.json']):
        guard.update(p.name.encode());guard.update(p.read_bytes())
    diff=subprocess.check_output(['git','diff','--name-only','HEAD'],cwd=ROOT,text=True).strip()
    reachability=read(OUT/'ihsid_reachability_v66.json')
    labels=read(OUT/'labels_v65/label_selection.json')
    summary={'formal':False,'formal_ready':False,'formal_runs':0,'formal_required':1440,
        'candidates':candidates,'runs':runs,'raw_failure_components':raw_counts,
        'recovery':recovery_summary(),'recovery_diagnostic':read(OUT/'recovery_diagnostic_v68.json'),
        'teacher_viability':read(OUT/'teacher_viability_diagnostic_ideal_v65.json'),
        'inference_matlab_single':read(OUT/'inference_v65.json'),'inference_python_bridge':read(OUT/'inference_v66.json'),
        'python_bridge_performance_accepted':False,
        'reachability':{'n':len(reachability['rows']),'necessary_infeasible':sum(r['bound']['infeasible'] for r in reachability['rows'])}
            if reachability else None,
        'label_selection':{'n':len(labels['rows']),'accepted':labels['accepted']} if labels else None,
        'model_training_boundary':'Seed 5101 only; 1006 train / 23 validation windows; moving-obstacle teacher coverage zero in current checkpoint.',
        'new_teacher_admitted':sum(r['admitted'] for r in runs['Baseline V67']['rows']),
        'new_teacher_used_for_training':False,
        'baseline_guard':guard.hexdigest(),'baseline_guard_matches':guard.hexdigest()=='a0735e0e37317afcf4107359797e9fbab71fcb886584beac318b42edbcb050d6',
        'tracked_changes':diff,'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'restoration':read(OUT/'finalized_v71.json'),'worker':read(CH5/'artifacts/performance_probe/worker_status.json'),
        'source_sha256':manifest}
    figures=[]
    if all(candidates.values()):
        fig,axes=plt.subplots(1,2,figsize=(9,3.5))
        for ax,key,label in zip(axes,['force_error_N','increment_error_N'],['Force-sequence error (N)','Increment error (N)']):
            ax.bar(list(candidates),[c[key]['median'] for c in candidates.values()],color=['#8797a5','#177f83'])
            ax.set_ylabel('Median '+label)
        fig.tight_layout();fig.savefig(OUT/'candidate_errors_v71.png',dpi=160);plt.close(fig)
        figures.append(('candidate_errors_v71.png','相同 92 个可比候选的标签误差。误差减少不等于原始候选可行。'))
    fig,ax=plt.subplots(figsize=(9,3.8));count=0
    for version in ['V60','V64','V68','V70','V71']:
        for task in [1,4]:
            row=read(VERSIONS[version]/f'{task:03d}_nonideal_force.json')
            if row and row['metrics'].get('simulated_end_seconds',15)>=15-1e-8:
                times=np.sort(np.asarray(row['refresh_seconds'])*1000)
                if len(times):ax.plot(times,np.arange(1,len(times)+1)/len(times),label=f'{version} task {task}');count+=1
    if count:
        ax.axvline(100,color='#b43d36',ls='--',label='100 ms');ax.set_xlabel('Complete planning time (ms)');ax.set_ylabel('Empirical CDF')
        ax.legend(fontsize=9,ncol=2);fig.tight_layout();fig.savefig(OUT/'planning_cdf_v71.png',dpi=160)
        figures.append(('planning_cdf_v71.png','相同完整 15 s 开发任务的规划耗时；包括生成、优化及物理检查。'))
    plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(10,3.5));counts=0
    for ax,task in zip(axes,[1,4]):
        rows=[(v,next((r for r in runs[v]['rows'] if r['task']==task and r['mode']=='nonideal-force' and r['episode_complete']),None))
            for v in ['V60','V64','V68','V70','V71']]
        rows=[(v,r) for v,r in rows if r is not None]
        ax.bar([v for v,r in rows],[r['wall_seconds'] for v,r in rows],color='#177f83');ax.set_title(f'Task {task}: 15 s simulation')
        ax.set_ylabel('Warm simulation wall time (s)');counts+=len(rows)
    if counts:
        fig.tight_layout();fig.savefig(OUT/'wall_time_v71.png',dpi=160)
        figures.append(('wall_time_v71.png','墙钟时间不含单独记录的模型更新和预热。动态任务未到达仍纳入。'))
    plt.close(fig)
    flat=[]
    for version,value in runs.items():
        for row in value['rows']:
            flat.append({'version':version,**{k:v for k,v in row.items() if not isinstance(v,(dict,list))},
                'refresh_median_ms':1000*row['refresh_seconds']['median'] if row.get('refresh_seconds') else None,
                'refresh_p95_ms':1000*row['refresh_seconds']['p95'] if row.get('refresh_seconds') else None})
    keys=list(dict.fromkeys(k for r in flat for k in r))
    with (OUT/'development_runs_v71.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(flat)
    (OUT/'summary_v71.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    (OUT/'source_manifest_v71.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    runrows=[[r['version'],r['task'],r['mode'],r['simulated_end_seconds'],r['all_constraints_valid'],r['arrival'],
        f"{r['final_position_mm']:.3f}",f"{r['wall_seconds']:.1f}",f"{r['refresh_median_ms']:.1f}" if r['refresh_median_ms'] else '—',
        r['fallback_count'],r.get('terminal_samples',0)] for r in flat]
    candidate_rows=[[name,f"{c['raw_accepted']}/{c['candidates']}",f"{c['projected_accepted']}/{c['candidates']}",
        f"{c['force_error_N']['median']:.3f}",f"{c['increment_error_N']['median']:.3f}"] for name,c in candidates.items() if c]
    infer=summary['inference_matlab_single'];bridge=summary['inference_python_bridge'];inference_rows=[]
    if infer:inference_rows+=[['MATLAB double',1000*infer['double_median_seconds'],'yes'],['MATLAB single',1000*infer['single_median_seconds'],'yes']]
    if bridge:inference_rows+=[['PyTorch kernel only',1000*bridge['kernel_median_seconds'],'not an end-to-end latency'],
        ['MATLAB → Python → MATLAB',1000*bridge['backend_median_seconds'],'no; retained failure']]
    recovery=summary['recovery'];recovery_rows=[]
    if recovery:recovery_rows=[[r['source'],r['time'],r['reset_to_current_brake'],r['initializer_accepted'],
        sum(r['proposal_accepted']),r['qp_success']] for r in recovery['rows']]
    pending=[name for name,v in runs.items() if v['pending'] and name in ['Baseline V67','V70','V71']]
    sections=["<h1>第五章开发验证：V62–V71</h1><p class='notice'>formal_ready=false；正式实验 0 / 1440。当前模型仅种子 5101，开发任务没有正式置信区间；所有失败和不完整运行保留。</p>",
        "<h2>已经实现</h2><p>QP 分解复用、同次规划的终端精确缓存、重复审计消除、0.1 s 失败重试节奏、数值热启动与可执行计划分离、部分敏感度更新、时序卷积扩散、MATLAB 单精度推理。V68 每次最多审计一个候选，其余明确标记为跳过；省时可能降低恢复能力，因此分别记录失败。</p>",
        "<p>控制增量始终经动力学产生预测轨迹；终端反馈使用增量律 u(k)=u(k−1)+Δu(k)。V70 只在静止目标、无活动障碍、紧容差到达且位于优化终端邻域时启用；每 20 ms 重新检查执行区间、执行器瞬态和数值 Lyapunov 下降。区域不是经过证明的安全或稳定证书，任务变化立即退出。</p>",
        "<p>文献依据：<a href='https://proceedings.mlr.press/v283/romer25a.html'>DPCC</a> 的约束预测；<a href='https://arxiv.org/html/2303.04137v5'>Diffusion Policy</a> 的时序卷积与条件化；<a href='https://arxiv.org/html/2406.06157v2'>MPC for tracking</a> 的辅助稳态与终端结构；<a href='https://docs.acados.org/features/index.html'>acados RTI</a> 的准备/反馈与部分更新思路。代码未使用 acados；项目组合没有直接继承这些文献的稳定性证明。</p>",
        "<h2>模型改善与可行性</h2><p>固定完整采样与标签：1006 个训练窗口、23 个验证窗口、20 步 DDIM、四候选。当前权重的移动障碍有效教师覆盖仍为零。新教师被接纳不意味着已加入当前权重训练。</p>",
        table(['模型','原始通过','优化通过','力误差中位数 N','增量误差中位数 N'],candidate_rows),
        "<p>时序模型原始加速度越界由 96/96 减少为 93/96；速度越界 94→90，行程越界 85→60，奇异性越界 90→48，输入变化率越界 7→0。原始通过仍为零，不能把优化后通过解释为扩散原始可行。</p>",
        "<h2>完整与短时 Simscape 结果</h2>",
        table(['版本','任务','执行器','实际仿真 s','全约束有效','到达','末位置 mm','墙钟 s','规划中位 ms','回退','终端步数'],runrows),
        "<p>初始模型更新/预热另存；15 s 仿真完成时间与墙钟计算时间分开。短时任务仅验证初段，不纳入完整教师。100 ms 是期望规划节奏，超过即如实报告，没有伪造实时性通过门槛。</p>",
        "<h2>基线参考与教师修复</h2><p>V65 仅加强单步输入保持约束，完整移动障碍任务仍失效。运行证据显示目标被移动障碍占据后 IHSID 重规划被拒绝，旧参考却一直被追踪。V67 对剩余旧 Hermite 参考按当前观测重审；无效时规划回任务初始安全位姿，仍保留原 IHSID、力前馈 LQI、严格 QP。没有可接受规划时明确记录制动参考不被证明可行。</p>",
        "<p>短时 train_004 两种执行器均全约束有效、零回退；完整任务的到达与安全结果见表。IHSID 必要的支链速度/加速度可达性界只提前拒绝必不可行请求，64 个独立 LP 检查通过；不据此声称充分可达。新教师只有完整安全成功才接纳。</p>",
        "<h2>失效后的恢复检查</h2><p>V68 强制拒绝窗口 [1.0,1.4) 后仍回退到 3 s；动态任务从 9.6 s 到 15 s 持续回退。四候选共享一个已偏离实际输入历史的数值热启动，后续 QP 多数不可行。V71 在固定失效状态比较原初始化与从实际机械/执行器状态构造的制动初始化；这是独立开发诊断，不是新增比较方法。</p>",
        table(['来源','时刻 s','重置初始化','初始化通过','提案通过 /4','QP成功 /4'],recovery_rows),
        "<h2>推理后端的负结果</h2>",table(['后端','中位耗时 ms','性能接纳'],inference_rows),
        "<p>MATLAB/Python 输出一致性通过，但跨进程往返约 2 s，已停止该试验并恢复 MATLAB 单精度控制。未重启 MATLAB，会话 PID 38888；不把内核 40 ms 当作完整调用耗时。</p>",
        "<h2>数据、复算与边界</h2><p>72 个固定窗口中原尾段修正和全时域修正各通过 70 个；对两种修正独立重审并择优后通过 71 个。这些择优标签尚未用于当前时序权重训练。原失败移动障碍教师没有混入训练。</p>",
        "<p><a href='summary_v71.json'>完整摘要</a> · <a href='development_runs_v71.csv'>开发运行 CSV</a> · <a href='source_manifest_v71.json'>源码哈希</a> · <a href='raw_audit_v65.json'>原始候选逐项越界</a> · <a href='recovery_diagnostic_v68.json'>回退区间诊断</a> · <a href='recovery_audit_v71.json'>恢复对比原始记录</a></p>",
        '<p>原 chapter5 顶层生产文件指纹一致：'+str(summary['baseline_guard_matches'])+'；跟踪文件改动：'+html.escape(diff or '无')+'。尚缺五种子正式训练、冻结任务与 1440 次正式实验。'+html.escape('待运行：'+', '.join(pending) if pending else '')+'</p>',
        "<pre>chapter5/.venv/Scripts/python.exe chapter5/reduced/force_v62/report_v71.py</pre>"]
    for file,caption in figures:sections.append(f"<figure><img src='{file}' alt='{html.escape(caption)}'><figcaption>{html.escape(caption)}</figcaption></figure>")
    css="body{font:16px/1.6 'Microsoft YaHei',sans-serif;max-width:1250px;margin:32px auto;padding:0 20px;color:#23313b}h1{font-size:28px}h2{font-size:22px;margin-top:30px}.notice{background:#fff3db;border-left:4px solid #c18b23;padding:12px}table{border-collapse:collapse;width:100%;font-size:13px;margin:16px 0}th,td{text-align:left;border-bottom:1px solid #d4dde3;padding:8px}th{background:#eaf0f3}img{max-width:100%;height:auto}figcaption{color:#596b76;font-size:14px}pre{overflow:auto;background:#edf2f4;padding:14px}a{color:#17657b}"
    (OUT/'index_v71.html').write_text("<!doctype html><html lang='zh-CN'><meta charset='utf-8'><title>Chapter 5 V62–V71 development</title><style>"+css+"</style><body>"+''.join(sections)+"</body></html>",encoding='utf-8')
    print('Report:',OUT/'index_v71.html');print('Production guard:',summary['baseline_guard_matches'])

if __name__=='__main__':main()
