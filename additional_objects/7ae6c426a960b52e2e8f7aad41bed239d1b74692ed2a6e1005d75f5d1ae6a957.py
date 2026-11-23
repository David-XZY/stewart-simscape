"""Recompute the isolated development report from saved numeric artifacts."""
from __future__ import annotations
import csv
import argparse
import hashlib
import html
import json
from pathlib import Path
import sys
import numpy as np
from scipy.io import loadmat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
PROBE = ROOT / 'chapter5' / 'artifacts' / 'reduced_probe'
OUT = PROBE / 'report'
OUT.mkdir(parents=True, exist_ok=True)
LATEST = 'closed_tapered_v9'
RECOVERY = 'recovery_tapered_v9'


def timing(a):
    a = np.asarray(a, dtype=float).ravel()
    return {'median': float(np.median(a)), 'p95': float(np.quantile(a, .95)),
            'max': float(np.max(a))} if a.size else {k: None for k in ('median', 'p95', 'max')}


def cells(value):
    return value if isinstance(value, list) else list(value) if isinstance(value, np.ndarray) else [value]


def ms(stats, key):
    return None if stats[key] is None else stats[key]*1000


def sources_hash():
    sys.path.insert(0, str(ROOT / 'chapter5'))
    import repair_campaign
    return repair_campaign.source_hash()


def summarize():
    rows, series, diagnostics = [], {}, {}
    for version in ('closed_feedback_v1', 'closed_latest_v2', 'closed_fast_v3', 'closed_factor_v4', 'recovery_factor_v4', 'closed_prior_v5', 'recovery_prior_v5', 'closed_normalized_v6', 'recovery_normalized_v6', 'closed_offset_v7', 'recovery_offset_v7', 'closed_reserved_v8', 'recovery_reserved_v8', 'closed_tapered_v9', 'recovery_tapered_v9', 'closed_multirate_v10', 'recovery_multirate_v10', 'closed_warm_v11', 'recovery_warm_v11'):
        for file in sorted((PROBE / version).glob('00*force.json')):
            r = json.loads(file.read_text('utf-8')); m = r['metrics']
            planner, executor = timing(r['refresh_seconds']), timing(r['execution_seconds'])
            row = dict(version=version, method='diffusion_control', task=r['task'], mode=r['mode'],
                       status='simulation_error' if r['simulation_error'] else 'completed' if m['simulated_end_seconds']>=14.999 else 'terminated',
                       arrival=m['arrival'], safe_success=m['safe_success'], tight_arrival=m['tight_arrival'],
                       constraints_valid=m['all_constraints_valid'], completed_seconds=m['completion_seconds'],
                       rearrival_seconds=json.dumps(m['rearrival_seconds']), position_final_mm=m['final_position'] * 1000,
                       rotation_final_deg=np.rad2deg(m['final_rotation']), wall_seconds=r['wall_seconds'], metrics_seconds=r.get('metrics_seconds'),
                       force_rms=m['actual_force_rms'], command_rate_rms=m['command_rate_rms'],
                       fallback_count=m['fallback_count'], solver_failure_count=m['solver_failure_count'],
                       predictive_solves=m['predictive_solves'], predictive_solver_failures=m['predictive_solver_failures'],
                       refresh_median_ms=ms(planner,'median'), refresh_p95_ms=ms(planner,'p95'),
                       refresh_max_ms=ms(planner,'max'), execution_median_ms=ms(executor,'median'),
                       execution_p95_ms=ms(executor,'p95'), execution_max_ms=ms(executor,'max'),
                       refresh_deadline_misses=r['refresh_deadline_misses'], execution_deadline_misses=r['execution_deadline_misses'],
                       minimum_clearance_mm=None if m['minimum_clearance'] is None else m['minimum_clearance'] * 1000,
                       simulation_error=r['simulation_error'], artifact=str(file.relative_to(ROOT)))
            rows.append(row)
            # Historical diagnostics need predictions only. Keep dense measured
            # series for the requested version, and never load unused records.
            variables=['predictions']
            if version==LATEST:variables.append('audit')
            mat = loadmat(file.with_suffix('.mat'), simplify_cells=True, variable_names=variables)
            if isinstance(mat.get('audit'), dict):
                series[(version, r['task'], r['mode'])] = mat['audit']
            stats, phases, controller_phases, rejections, lines = {}, [], [], 0, []
            rejection_causes = dict(terminal=0, mechanics=0, collision=0, transient=0, command_rate=0, nonfinite=0,
                                    injected=0, no_retained_plan=0, algebraic_input_constraint=0)
            scopes, duplicate_skips = {}, 0
            for prediction in cells(mat.get('predictions', [])):
                p = prediction['prediction']
                scope=p.get('validation_scope','full_horizon')
                scopes[scope]=scopes.get(scope,0)+1
                duplicate_skips+=int(p.get('duplicate_audits_skipped',0))
                if isinstance(p.get('phases'), dict) and p['refresh']: controller_phases.append(p['phases'])
                if not p.get('chosen', 0):
                    rejections += 1
                    for check in cells(p.get('checks', [])):
                        if not isinstance(check, dict): continue
                        rejection_causes['terminal'] += check.get('equality_tolerance_ratio', 0) > 1
                        internal=check.get('internal_check', {})
                        rejection_causes['mechanics'] += internal.get('mechanical_violation', 0) > 1e-6
                        rejection_causes['collision'] += internal.get('collision_violation', 0) > 1e-6
                        rejection_causes['transient'] += check.get('transient', {}).get('maximum_violation', 0) > 1e-6
                        rejection_causes['command_rate'] += check.get('command_rate_violation', 0) > 1e-6
                        rejection_causes['nonfinite'] += not check.get('finite', True)
                        rejection_causes['injected'] += check.get('rejection_reason') == 'development_fault_injection'
                        for name in ('no_retained_plan','algebraic_input_constraint'):
                            rejection_causes[name] += check.get('rejection_reason') == name
                for attempt in cells(p.get('denoise_attempts', [])):
                    if not isinstance(attempt, dict): continue
                    status = attempt.get('solver_status', attempt.get('status', 'unknown'))
                    stats[status] = stats.get(status, 0) + 1
                    if isinstance(attempt.get('phases'), dict): phases.append(attempt['phases'])
                    if 'line' in attempt: lines.append(attempt['line'])
            phase_stats = {k: timing([p[k] for p in phases]) for k in phases[0]} if phases else {}
            diagnostics[str(file.relative_to(ROOT))] = dict(solver_statuses=stats,
                no_executable_candidate_samples=rejections, phase_seconds=phase_stats,
                overlapping_rejection_causes={name:int(count) for name,count in rejection_causes.items()},
                validation_scopes=scopes, exact_duplicate_audits_skipped=duplicate_skips,
                controller_phase_seconds={k: timing([p[k] for p in controller_phases]) for k in controller_phases[0]} if controller_phases else {},
                backtracking_lines={str(i): lines.count(i) for i in sorted(set(lines))})
    # Original admitted baseline is paired only by exact task identity, actuator
    # and unchanged root source hash. It is historical development evidence.
    paired_tasks = sorted({(r['task'], r['mode']) for r in rows if r['version'] == LATEST})
    baseline = []
    native_inputs=json.loads((PROBE/'report_audit_v1'/'summary.json').read_text('utf-8'))
    task_file=ROOT/'chapter5/artifacts/tasks/development.mat'
    assert hashlib.sha256(task_file.read_bytes()).hexdigest()==native_inputs['task_file_sha256'], 'Stale MATLAB task export'
    tasks=native_inputs['tasks']
    for task, mode in paired_tasks:
        file = ROOT / 'chapter5' / 'artifacts' / 'development' / f'development_{task:03d}_{mode.replace("-", "_")}_baseline_0.json'
        if not file.exists(): continue
        r = json.loads(file.read_text('utf-8')); m = r.get('metrics')
        if not m: continue
        baseline_files=r['source_identity']['files']
        mismatches=[]
        for name,digest in baseline_files.items():
            stem,extension=name.rsplit('_',1); current=ROOT/'chapter5'/f'{stem}.{extension}'
            if not current.exists() or hashlib.sha256(current.read_bytes()).hexdigest()!=digest:mismatches.append(name)
        current_task=tasks[task-1]
        task_matches=all(name in current_task and np.array_equal(np.asarray(value).ravel(),np.asarray(current_task[name]).ravel())
                         for name,value in r['task'].items())
        config_matches=r['source_identity']['baseline_source_sha256']==native_inputs['source_identity']['baseline_source_sha256']
        paired_eligible=not mismatches and task_matches and config_matches
        baseline.append(dict(task=task, mode=mode, method='baseline', historical=True,
            safe_success=m['safe_success'], completed_seconds=m['completion_seconds'],
            final_position_mm=m['final_position'] * 1000, actual_force_rms=m['actual_force_rms'],
            command_rate_rms=m['command_rate_rms'], wall_seconds=r['wall_seconds'],
            initial_planning_seconds=m['initial_planning_seconds'], controller_seconds=m['controller_seconds'],
            source_identity=r['source_identity'], source_file_mismatches=mismatches,
            task_matches_current=task_matches, baseline_config_matches=config_matches,
            paired_eligible=paired_eligible, artifact=str(file.relative_to(ROOT))))
        mat_file = file.with_suffix('.mat')
        if mat_file.exists() and paired_eligible:
            data = loadmat(mat_file, simplify_cells=True, variable_names=['audit'])
            if isinstance(data.get('audit'), dict): series[('baseline', task, mode)] = data['audit']
    with (OUT / 'closed_loop.csv').open('w', encoding='utf-8-sig', newline='') as f:
        if rows:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (OUT / 'diagnostics.json').write_text(json.dumps(diagnostics, ensure_ascii=False, indent=2), 'utf-8')
    (OUT / 'paired_baseline.json').write_text(json.dumps(baseline, ensure_ascii=False, indent=2), 'utf-8')
    development_pairs=[]
    for current in (r for r in rows if r['version']==LATEST):
        old=next((r for r in rows if r['version']=='closed_tapered_v9' and (r['task'],r['mode'])==(current['task'],current['mode'])),None)
        if old is None:continue
        pair=dict(task=current['task'],mode=current['mode'],previous_version=old['version'],current_version=LATEST)
        for name in ('safe_success','wall_seconds','metrics_seconds','refresh_median_ms','refresh_p95_ms','refresh_max_ms',
                     'execution_median_ms','execution_p95_ms','execution_max_ms','fallback_count','predictive_solver_failures'):
            pair[f'previous_{name}']=old[name];pair[f'current_{name}']=current[name]
        development_pairs.append(pair)
    with (OUT/'development_versions.csv').open('w',encoding='utf-8-sig',newline='') as f:
        if development_pairs:
            writer=csv.DictWriter(f,fieldnames=list(development_pairs[0]));writer.writeheader();writer.writerows(development_pairs)
    plot(rows, series)
    source_files = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*') if p.is_file()}
    resources = [json.loads(l) for l in (PROBE / 'resources.jsonl').read_text('utf-8').splitlines() if l.strip()]
    peak = max((r['private_bytes'] for r in resources), default=0) / 2**30
    factor_audit=json.loads((PROBE/'factor_stage_audit_v1'/'summary.json').read_text('utf-8'))
    factor_decision=dict(accepted=factor_audit['accepted'],
        original_median_ms=np.median(factor_audit['original_seconds'])*1000,
        factor_median_ms=np.median(factor_audit['factor_seconds'])*1000,
        enabled=False, reason='Equivalent but slower; retain original derivative kernel')
    thread_audit=json.loads((PROBE/'thread_audit_v1'/'summary.json').read_text('utf-8'))
    thread_rows=[dict(mode=row['mode'],threads=row['threads'],sample=row['sample'],
        serial_stage_ms=float(np.median(row['serial_seconds'][0])*1000),
        thread_stage_ms=float(np.median(row['thread_seconds'][0])*1000),
        serial_mechanics_ms=float(np.median(row['serial_seconds'][1])*1000),
        thread_mechanics_ms=float(np.median(row['thread_seconds'][1])*1000),
        accepted=row['accepted']) for row in thread_audit['rows']]
    thread_decision=dict(accepted=thread_audit['accepted'],enabled=False,
        reason='Equivalent numerical results and QP matrices, but all measured thread configurations were slower',rows=thread_rows)
    multirate_audit=None
    if LATEST!='closed_tapered_v9':
        audit_name='warm_audit_v11' if LATEST=='closed_warm_v11' else 'multirate_audit_v10'
        multirate_audit=json.loads((PROBE/audit_name/'summary.json').read_text('utf-8'))
    archive=PROBE/f'{LATEST}_source.zip'
    source_consistency=archive.exists()
    if archive.exists():
        import zipfile
        with zipfile.ZipFile(archive) as zipped:
            expected={Path(name).name.replace('.','_'):hashlib.sha256(zipped.read(name)).hexdigest() for name in zipped.namelist() if name.endswith('.m')}
        for file in (PROBE/LATEST).glob('00*force.json'):
            case_identity=json.loads(file.read_text('utf-8'))['code_identity']
            source_consistency=source_consistency and all(case_identity.get(name)==digest for name,digest in expected.items())
    conclusion = dict(production_source_hash=sources_hash(), source_files=source_files,
        formal_ready=False, formal_runs=len(list((ROOT / 'chapter5/artifacts/formal').glob('*.mat'))),
        scope='Development only; one training seed; versions kept separate',
        latest_version=LATEST, latest_runs=sum(r['version'] == LATEST for r in rows),
        latest_safe_successes=sum(r['version'] == LATEST and r['safe_success'] for r in rows),
        latest_complete=sum(r['version']==LATEST and r['status']=='completed' for r in rows)==6,
        latest_refresh_deadline_misses=sum(r['refresh_deadline_misses'] for r in rows if r['version']==LATEST),
        latest_execution_deadline_misses=sum(r['execution_deadline_misses'] for r in rows if r['version']==LATEST),
        latest_recovery_runs=sum(r['version']==RECOVERY for r in rows),
        unadopted_offset_variant=dict(version='closed_offset_v7',reason='Moving-obstacle arrival did not improve; epigraph QP introduced additional solver failures',
            completed_runs=sum(r['version']=='closed_offset_v7' for r in rows),
            predictive_solver_failures=sum(r['predictive_solver_failures'] for r in rows if r['version']=='closed_offset_v7')),
        factor_derivative_probe=factor_decision,
        parallel_derivative_probe=thread_decision,
        multirate_audit=None if multirate_audit is None else dict(accepted=multirate_audit['accepted'],
            old_full_horizon_rejections=multirate_audit['full_horizon_rejections_replayed'],
            old_rejected_first_holds_valid=multirate_audit['replayed_first_holds_valid'],
            old_transient_rejected=multirate_audit['old_transient_rejected']),
        source_archive_matches_all_latest_cases=source_consistency,
        resource_peak_process_tree_private_gib=peak,
        deadline_definition='complete refresh <=100 ms; intervening complete execution <=20 ms',
        solver_and_candidate_statuses_are_separate=True, rows=rows)
    conclusion_file=PROBE/('conclusion.json' if LATEST=='closed_tapered_v9' else f'conclusion_{LATEST}.json')
    conclusion_file.write_text(json.dumps(conclusion, ensure_ascii=False, indent=2), 'utf-8')
    (OUT/'conclusion.json').write_text(json.dumps(conclusion, ensure_ascii=False, indent=2),'utf-8')
    write_html(rows, baseline, peak, thread_rows)
    print(json.dumps({k: v for k, v in conclusion.items() if k not in ('rows', 'source_files', 'parallel_derivative_probe')}, ensure_ascii=False))


def plot(rows, series):
    latest = [r for r in rows if r['version'] == LATEST]
    if not latest: return
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    labels = [f"T{r['task']} {'ideal' if r['mode']=='ideal-force' else 'nonideal'}" for r in latest]
    x = np.arange(len(latest))
    for ax, prefix, budget in zip(axes, ('refresh', 'execution'), (100, 20)):
        ax.bar(x-.17, [r[f'{prefix}_median_ms'] for r in latest], .34, label='median')
        ax.bar(x+.17, [r[f'{prefix}_p95_ms'] for r in latest], .34, label='P95')
        ax.axhline(budget, color='firebrick', linestyle='--', label=f'{budget} ms budget')
        ax.set_xticks(x, labels, rotation=25);ax.set_ylabel('complete computation (ms)');ax.set_title(prefix);ax.legend(fontsize=8)
    fig.savefig(OUT / 'timing.png', dpi=180);plt.close(fig)
    for task in sorted({r['task'] for r in latest}):
        fig, axes = plt.subplots(2, 2, figsize=(10, 6), constrained_layout=True)
        for mode in ('ideal-force', 'nonideal-force'):
            a = series.get((LATEST, task, mode))
            if a is None: continue
            t = np.asarray(a['time']).ravel();label=mode.replace('-force','')
            axes[0,0].plot(t, np.asarray(a['position_error']).ravel()*1000, label=label)
            axes[0,1].plot(t, np.rad2deg(np.asarray(a['rotation_error']).ravel()), label=label)
            acceleration = np.maximum(np.max(np.abs(a['leg_acceleration_measured']), axis=1),
                np.max(np.abs(a['leg_acceleration_model_actual_force']), axis=1))
            axes[1,0].plot(t, acceleration, label=label)
            axes[1,1].plot(t, np.linalg.norm(a['actual_force'], axis=1)/np.sqrt(6), label=label)
            np.savez_compressed(OUT / f'task_{task:03d}_{label}.npz', **{k: v for k,v in a.items() if isinstance(v, np.ndarray)})
        for mode in ('ideal-force',):
            b = series.get(('baseline', task, mode))
            if b is not None:
                t=np.asarray(b['time']).ravel()
                axes[0,0].plot(t,np.asarray(b['position_error']).ravel()*1000,'--',label='baseline ideal (historical)')
                axes[0,1].plot(t,np.rad2deg(np.asarray(b['rotation_error']).ravel()),'--',label='baseline ideal (historical)')
        axes[0,0].set_ylabel('position error (mm)');axes[0,1].set_ylabel('rotation error (deg)')
        axes[0,0].axhline(5,color='firebrick',linestyle=':',label='position tolerance')
        axes[0,1].axhline(.3,color='firebrick',linestyle=':',label='rotation tolerance')
        axes[1,0].set_ylabel('max leg acceleration, measured/model (m/s^2)');axes[1,1].set_ylabel('actual force RMS across legs (N)')
        axes[1,0].axhline(1.2,color='firebrick',linestyle='--',label='physical limit')
        for ax in axes.ravel():ax.set_xlabel('simulation time (s)');ax.legend(fontsize=7);ax.grid(alpha=.25)
        fig.suptitle(f'Development task {task}: complete physical closed loop')
        fig.savefig(OUT / f'task_{task:03d}.png',dpi=180);plt.close(fig)
        fig, ax=plt.subplots(figsize=(6,5),constrained_layout=True)
        for mode in ('ideal-force','nonideal-force'):
            a=series.get((LATEST,task,mode))
            if a is None:continue
            pose=np.asarray(a['absolute_pose']);ax.plot(pose[:,0]*1000,pose[:,1]*1000,label=mode.replace('-force',''))
            ax.scatter(pose[0,0]*1000,pose[0,1]*1000,marker='o',s=30)
            ax.scatter(pose[-1,0]*1000,pose[-1,1]*1000,marker='x',s=40)
        ax.set_aspect('equal',adjustable='datalim');ax.set_xlabel('world x (mm)');ax.set_ylabel('world y (mm)')
        ax.set_title(f'Task {task}: measured XY motion');ax.legend();ax.grid(alpha=.25)
        fig.savefig(OUT/f'motion_{task:03d}.png',dpi=180);plt.close(fig)


def write_html(rows, baseline, peak, thread_rows):
    latest=[r for r in rows if r['version']==LATEST]
    recovery=[r for r in rows if r['version']==RECOVERY]
    recovery_text=''.join(f"<p>恢复测试：任务 {r['task']}，{r['mode']}，状态 {r['status']}，安全成功 {r['safe_success']}，回退 {r['fallback_count']} 次，最终位置误差 {r['position_final_mm']:.3f} mm。</p>" for r in recovery)
    def fmt(v):return '—' if v is None else f'{v:.2f}'
    body=''.join('<tr>'+''.join(f'<td>{html.escape(str(v))}</td>' for v in (
        r['task'],r['mode'],r['safe_success'],fmt(r['completed_seconds']),fmt(r['position_final_mm']),
        fmt(r['wall_seconds']),fmt(r['metrics_seconds']),fmt(r['refresh_median_ms']),fmt(r['refresh_p95_ms']),
        fmt(r['execution_median_ms']),fmt(r['execution_p95_ms']),fmt(r['execution_max_ms']),
        r['refresh_deadline_misses'],r['execution_deadline_misses'],
        r['predictive_solver_failures'],r['fallback_count']))+'</tr>' for r in latest)
    version_body=''
    for current in latest:
        for version in ('closed_tapered_v9','closed_multirate_v10','closed_warm_v11'):
            previous=next((r for r in rows if r['version']==version and (r['task'],r['mode'])==(current['task'],current['mode'])),None)
            if previous is None:continue
            version_body+='<tr>'+''.join(f'<td>{html.escape(str(v))}</td>' for v in (
                current['task'],current['mode'],version,previous['safe_success'],fmt(previous['wall_seconds']),
                fmt(previous['refresh_p95_ms']),fmt(previous['execution_p95_ms']),
                previous['predictive_solver_failures'],previous['fallback_count']))+'</tr>'
    thread_table=''.join('<tr>'+''.join(f'<td>{html.escape(str(v))}</td>' for v in (
        r['mode'],r['threads'],r['sample'],fmt(r['serial_stage_ms']),fmt(r['thread_stage_ms']),
        fmt(r['serial_mechanics_ms']),fmt(r['thread_mechanics_ms']),r['accepted']))+'</tr>' for r in thread_rows)
    paired_count=sum(r['paired_eligible'] for r in baseline)
    figures=''.join(f'<h2>开发任务 {task}</h2><img src="task_{task:03d}.png"><img style="max-width:600px" src="motion_{task:03d}.png">' for task in sorted({r['task'] for r in latest}))
    text=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>控制增量扩散：开发验证</title>
<style>body{{font:16px/1.65 system-ui;max-width:1200px;margin:35px auto;padding:0 20px;color:#202936}}table{{border-collapse:collapse;width:100%}}td,th{{border-bottom:1px solid #ccd4df;padding:8px}}img{{width:100%}}.note{{background:#edf3fa;padding:16px}}</style>
<h1>控制增量扩散：降维、闭环与计算验证</h1>
<p class="note">开发结果，训练种子 5101。各版本单独保留，不作为 1440 次正式实验或总体成功率结论。
全部时间为本机实测。15 s 是仿真时间；墙钟时间及 100/20 ms 预算另行列出。当前表格版本：{LATEST}。</p>
<h2>本次修改</h2><p>保留控制增量与原执行器增广动力学，使用 72 个分块修正坐标和数值反馈凝聚。
保留完整旧序列，按原终端数值容差决定是否修正尾段。全部内部采样点继续检查原物理约束。
四条扩散先验在去噪后排名，本提速原型优化其中一条；并非原正式计划的四条各自优化。
先验代价为采样控制增量的二次偏差项，不宣称计算了扩散模型的精确对数概率。</p>
<p>复用已安装模型和执行器变体，跳过旧跟踪设计的重复线性化。物理初始条件仍逐任务重置。
理想→非理想→理想的短时施力响应已独立核对。下一拍位置误差低于 0.1 μm。</p>
<p>制动初值与恢复采用支链空间加速度限幅，回退后检查完整 20 ms 保持区间。
QP 成功、有限网格检查或回退均不构成鲁棒安全证明。</p>
<p>版本 v2 的非理想目标切换任务在启动阶段 0.140105 s 曾发生真实瞬态超限：最大 0.047128 m/s²，
持续约 0.105 ms。此失败完整保留，未并入 v3 的结果。
版本 v3 在每个预测区间追加 25–200 μs 检查点及六支链斜坡结束时刻；
独立重放成功拒绝了先前漏检的指令。该有限网格细化仍不构成连续时间安全保证。</p>
<p>版本 v4 修正后验代价检查中先验被意外置零的参数判断，使 QP 与后验代价采用同一先验项。
非理想动力学导数尝试用小步 RK4 的精确链式法则计算；先通过独立原值、状态/输入雅可比与非零先验代价检查再进入闭环。
审计耗时单列，不算作在线控制计算。另有 0.4–0.6 s 强制候选拒绝的恢复测试，单独保留。</p>
<p>小步导数试验通过等价性核对，但本机实测更慢，最终保留原导数实现。
版本 v4 又暴露出间隔控制步传入空先验的接口错误，启动失败记录完整保留。
版本 v5 修复了这个问题并通过连续两拍调用检查，但实际到达指标未通过。
版本 v6 将二次先验代理与其他运行代价一样按控制周期积分，保留非零权重；
该调整是当前原型的时间尺度校准，不宣称对应扩散模型精确对数密度。</p>
<p>v6 计时还纳入坐标变换、执行器状态更新、任务观测和当前物理检查。
各旧版本计时范围略小，耗时比较保留这个差别；全部计时均不包含离线审计与日志落盘。</p>
<p>v6 的移动障碍任务仍未满足低速连续保持判据，保留到达失败。
v7 在稳态偏置二次项上增加 L1 项，借鉴 MPC for tracking 对线性增长偏置代价的讨论（文中 §II-D）。
QP 增加六个绝对值上界辅助变量：共 78 个变量，其中 72 个为物理输入的修正坐标。
实测未解决移动障碍的到达失败，并增加 QP 求解失败；故不采用 v7。
终端位姿仍是控制动力学展开结果；没有要求短时域必须到达目标。其原系统理论并不直接构成此非线性、移动障碍、学习组合的保证。</p>
<p>v8 针对进一步核对出的失败原因：QP 迭代上限从 100 提高到 400；
仅在线性化碰撞约束中增加 0.2 mm 余量，处理原端点之间微米级的裕度不足；
速度运行代价权重从 0.1 提高到 1。原非线性硬约束检查和到达判据不变。
这是开发期的工程设置，不能据此宣称连续安全或实时保证。</p>
<p>v8 增加固定余量后，当前已确定的状态也被要求满足额外余量，产生新的结构性不可行。
v9 将额外余量改成 0.2 mm × min((预测时间/0.1 s)², 1)，在不可改变的当前状态为零，随后平滑增加。
原非线性完整采样网格仍按同一阈值验收；不会因为 QP 成功而放行物理检查失败的候选。
其他版本均保留于完整 CSV。</p>
<p>v10 将完整预测验收保留在 10 Hz 刷新，将 50 Hz 间隔步改成当前状态下的 20 ms 执行区间检查。
原硬限制、原积分采样点和执行器瞬态点仍用于实际执行区间；间隔步不宣称整段旧预测仍满足约束。
每次刷新仍检查完整 1.2 s 轨迹及终端条件。输入代数约束可提前拒绝；本次调用中完全相同的序列不重复验收。
v10 完整保留了理想移动障碍回退由 2 增至 26 次的退步。</p>
<p>v11 单独恢复 v9 的逐拍动力学初值推进与必要的尾段数值修正，但保留 v10 的执行区间验收。
更新后的整段序列明确标为数值初值，不标为已经完整验收的未来轨迹。
独立检查同时核对执行区间原值、约束、刷新时钟、旧瞬态拒绝，以及名义状态下与 v9 首个力指令的一致性。</p>
<h2>最新完整闭环结果</h2><div style="overflow-x:auto"><table><tr><th>任务</th><th>执行器</th><th>安全成功</th><th>到达时刻 s</th><th>最终位置 mm</th><th>15 s 仿真墙钟 s</th><th>离线审计 s</th><th>规划中位 ms</th><th>规划 P95 ms</th><th>执行中位 ms</th><th>执行 P95 ms</th><th>执行最大 ms</th><th>规划超时次数</th><th>执行超时次数</th><th>预测 QP 失败</th><th>回退</th></tr>{body}</table></div>
<p>任务 1：固定目标；任务 3：4 s 时目标切换；任务 4：连续移动障碍。到达还要求速度足够小并保持 0.5 s。
所有失败、约束违反、求解器状态和候选拒绝均见原始数据与 CSV。</p><img src="timing.png">
<h2>同任务开发版本变化</h2><p>下面仅用于定位开发改动的效果；同一任务的重复运行不作为新增独立样本，也没有按工况拼接选择不同版本。
各版本均在同一持久 MATLAB 进程中运行。墙钟时间不含模型更新、首次编译和离线审计；这些耗时分别保存在原始结果中。
P95 达标不代表每次计算都达标，超时次数及最大值也保留。</p>
<div style="overflow-x:auto"><table><tr><th>任务</th><th>执行器</th><th>开发版本</th><th>安全成功</th><th>15 s 仿真墙钟 s</th><th>规划 P95 ms</th><th>执行 P95 ms</th><th>预测 QP 失败</th><th>回退</th></tr>{version_body}</table></div>
{figures}<h2>独立导数计算的并行试验</h2>
<p>按 <a href="https://web.casadi.org/docs/#map">CasADi 官方 ThreadMap 接口</a>测试 2、4、8 线程。
两种执行器、两个保存状态共 12 组，全部原值、雅可比和凝聚 QP 矩阵一致。
本机所有线程配置均更慢，因此不启用。状态递推没有并行化，也没有减少物理检查。</p>
<table><tr><th>执行器</th><th>线程</th><th>样本</th><th>串行导数 ms</th><th>并行导数 ms</th><th>串行机械约束 ms</th><th>并行机械约束 ms</th><th>等价检查</th></tr>{thread_table}</table>
<h2>强制拒绝与恢复</h2>{recovery_text}<h2>统计与复算边界</h2><p>历史基线按同一开发任务和执行器核对，来源身份另存。
已逐文件核对原基线代码哈希、登记的基线配置身份和任务各字段：{paired_count}/{len(baseline)} 条可配对。
任务经 MATLAB 原生导出，避免将 string 数组误读为 MCOS 占位对象。来源或任务不一致的记录不进入配对曲线。
本次只验证少量开发任务，不给出正式置信区间，不据此断言新方法总体优于基线。
初始规划、任务完成、刷新计算和间隔执行分别统计。</p>
<p>本进程树观测峰值私有内存 {peak:.2f} GiB；该指标包含构建、求解及 MATLAB 子进程。
此轮未观察到内存不足，不能据此保证完整正式任务规模的内存表现。</p>
<p>方法依据：<a href="https://arxiv.org/abs/2109.12355">input move-blocking</a>；
<a href="https://proceedings.mlr.press/v283/romer25a.html">DPCC</a>；
<a href="https://arxiv.org/html/2406.06157v2">MPC for tracking</a>。
这些文献的稳定性、可行性保证不直接移用于本组合系统。</p>
<p><a href="closed_loop.csv">完整开发结果 CSV</a> · <a href="diagnostics.json">求解诊断及分阶段耗时</a> ·
<a href="development_versions.csv">同任务开发版本变化</a> ·
<a href="paired_baseline.json">历史配对基线来源</a> · <a href="conclusion.json">源代码哈希与完成边界</a></p></html>'''
    (OUT / 'index.html').write_text(text,'utf-8')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version',choices=('closed_tapered_v9','closed_multirate_v10','closed_warm_v11'),default=LATEST)
    args=parser.parse_args();LATEST=args.version
    RECOVERY={'closed_tapered_v9':'recovery_tapered_v9','closed_multirate_v10':'recovery_multirate_v10','closed_warm_v11':'recovery_warm_v11'}[LATEST]
    OUT=PROBE/({'closed_tapered_v9':'report','closed_multirate_v10':'report_v10','closed_warm_v11':'report_v11'}[LATEST])
    OUT.mkdir(parents=True,exist_ok=True);summarize()
