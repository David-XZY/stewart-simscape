"""Recomputable paired, task-clustered, training-seed-stratified summaries."""
from __future__ import annotations

import csv
import argparse
import html
import json
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import scipy.io

ROOT = Path(__file__).resolve().parent
ART = ROOT / 'artifacts'
OUT = ART / 'report'
FAMILIES = {1: 'Unseen start and goal', 2: 'Clutter and narrow passage', 3: 'Goal changes', 4: 'Moving obstacles'}


def number(value):
    return np.nan if value is None else float(value)


def event_rows(raw):
    """One row per observed task change; retain unreached, censored events."""
    metric = raw.get('metrics', {})
    task = raw['task']
    base = {'id': task['id'], 'family': task['family'], 'difficulty': task['difficulty'],
            'actuator': raw['mode'], 'method': raw['method'], 'seed': raw['seed'],
            'version': raw['version'], 'status': raw['status']}
    end = number(metric.get('simulated_end_seconds'))
    if not np.isfinite(end):
        end = 0.  # No returned trajectory is not an observed 15-second failure.
    goal_times = np.atleast_1d(task.get('goal_times', [0])).astype(float)
    goal_delay = np.atleast_1d(metric.get('rearrival_seconds', [])).tolist()
    result = []
    for i in range(1, len(goal_times)):
        event_time = float(goal_times[i])
        delay = number(goal_delay[i]) if i < len(goal_delay) else np.nan
        until = min(end, goal_times[i + 1]) if i + 1 < len(goal_times) else end
        result.append({**base, 'event_kind': 'goal_change', 'event_index': i, 'event_seconds': event_time,
                       'observed': int(event_time <= end), 'reached': int(np.isfinite(delay)),
                       'rearrival_seconds': delay, 'observed_followup_seconds': max(0., until - event_time)})
    times = np.atleast_1d(metric.get('obstacle_goal_clear_times', [])).tolist()
    delays = np.atleast_1d(metric.get('obstacle_rearrival_seconds', [])).tolist()
    for i, event_time in enumerate(times):
        delay = number(delays[i]) if i < len(delays) else np.nan
        result.append({**base, 'event_kind': 'obstacle_goal_clear', 'event_index': i,
                       'event_seconds': float(event_time), 'observed': 1, 'reached': int(np.isfinite(delay)),
                       'rearrival_seconds': delay, 'observed_followup_seconds': max(0., end - float(event_time))})
    return result


def planning_diagnostics(file, sample=None):
    """Separate geometric rejection without a solver call from solver failure."""
    if sample is None:
        sample = scipy.io.loadmat(file, simplify_cells=True, variable_names=['plans'])
    plans = sample.get('plans', [])
    if isinstance(plans, dict):
        plans = [plans]
    counts = {'planning_solver_calls': 0, 'planning_solver_failures': 0, 'planning_precheck_rejections': 0}
    for plan in np.atleast_1d(plans):
        attempts = plan['plan'].get('attempts', [])
        if isinstance(attempts, dict):
            attempts = [attempts]
        for attempt in np.atleast_1d(attempts):
            called = bool(attempt.get('solver_called', True))
            counts['planning_solver_calls'] += int(called)
            counts['planning_solver_failures'] += int(called and not attempt.get('solver_success', False))
            counts['planning_precheck_rejections'] += int(not called)
    return counts


def control_diagnostics(file, raw, config):
    """Recompute executed-command statistics, including the first increment.

    A final output computed at the simulation stop time has no executed
    interval. Keep its computational cost, but omit it from input statistics.
    The stored legacy metric is preserved separately in the episode CSV.
    """
    sample = scipy.io.loadmat(file, simplify_cells=True, variable_names=['records', 'plans'])
    records = sample.get('records', [])
    if isinstance(records, dict):
        records = [records]
    records = list(np.atleast_1d(records))
    result = planning_diagnostics(file, sample)
    timings = {'controller': [], 'initial_planning': [], 'replanning': []}
    if not records:
        return result, timings
    times = np.array([record['time'] for record in records], dtype=float)
    assert np.all(np.diff(times) > 0), 'Control records must be strictly chronological.'
    timings['controller'] = [float(record['controller_seconds']) for record in records]
    timings['initial_planning'] = [float(records[0]['planning_seconds'])] if records[0]['planning_seconds'] > 0 else []
    timings['replanning'] = [float(record['planning_seconds']) for record in records[1:] if record['planning_seconds'] > 0]
    end = number(raw.get('metrics', {}).get('simulated_end_seconds'))
    if not np.isfinite(end):
        end = times[-1]  # Report only observed intervals of a failed run.
    durations = np.maximum(0., np.minimum(np.r_[times[1:], end], end) - times)
    executed = durations > 1e-10
    commands = np.stack([record['u'] for record in records]).astype(float)
    previous = np.stack([record['previous_u'] for record in records]).astype(float)
    rates = (commands - previous) / config['Ts']
    # Read the exported shared physical configuration; do not infer bounds
    # from the distribution of observed commands or duplicate constants.
    limits = json.loads((ART / 'validation' / 'physical_parameters.json').read_text('utf-8'))
    assert limits['passed'] and limits['version'] == raw['version'] and limits['Ts'] == config['Ts']
    force_lower = np.asarray(limits['force_min'], dtype=float)
    force_upper = np.asarray(limits['force_max'], dtype=float)
    rate_limit = np.asarray(limits['force_rate_limit'], dtype=float)
    force_violation = np.maximum(0., np.max(np.maximum(commands - force_upper, force_lower - commands), axis=1))
    rate_violation = np.maximum(0., np.max(np.abs(rates) - rate_limit, axis=1))
    for name, violation in [('command_force', force_violation), ('command_rate', rate_violation)]:
        result[f'violation_{name}_seconds'] = float(durations[violation > 1e-6].sum())
        result[f'violation_{name}_maximum'] = float(violation[executed].max()) if executed.any() else np.nan
    result['command_rate_rms'] = float(np.sqrt((rates ** 2 * durations[:, None]).sum() / (6 * durations.sum()))) if durations.sum() else np.nan
    result['executed_control_intervals'] = int(executed.sum())
    return result, timings


def pooled_compute_statistics(groups):
    result = []
    for (method, seed, actuator), categories in sorted(groups.items()):
        for category, values in categories.items():
            values = np.asarray(values, dtype=float)
            assert np.all(np.isfinite(values)) and np.all(values >= 0)
            result.append({'method': method, 'seed': seed, 'actuator': actuator, 'calculation': category,
                           'calls': len(values), 'median_seconds': float(np.median(values)) if len(values) else np.nan,
                           'p95_seconds': float(np.quantile(values, .95)) if len(values) else np.nan,
                           'maximum_seconds': float(values.max()) if len(values) else np.nan})
    return result


def rows(include_diagnostics=False, timing_groups=None):
    result = []
    for file in sorted((ART / 'formal').glob('*.json')):
        raw = json.loads(file.read_text('utf-8'))
        assert file.with_suffix('.mat').exists(), f'Missing raw episode: {file}'
        task = raw['task']
        metric = raw.get('metrics', {})
        row = {'id': task['id'], 'family': task['family'], 'difficulty': task['difficulty'],
               'actuator': raw['mode'], 'method': raw['method'], 'seed': raw['seed'],
               'version': raw['version'], 'status': raw['status'], 'file': file.with_suffix('.mat').name,
               'arrival': int(metric.get('arrival', False)), 'safe_success': int(metric.get('safe_success', False)),
               'tight_arrival': int(metric.get('tight_arrival', False)), 'collision': int(metric['collision']) if 'collision' in metric else np.nan,
               'wall_seconds': raw.get('wall_seconds'), 'error': raw.get('error', {}).get('message', '')}
        for name in ['completion_seconds', 'position_rms', 'rotation_rms', 'final_position', 'final_rotation',
                     'actual_force_rms', 'command_rate_rms', 'minimum_clearance', 'fallback_count', 'solver_failure_count',
                     'planning_attempts', 'planning_rejections', 'predictive_solves', 'predictive_solver_failures']:
            row[name] = number(metric.get(name))
        row['planning_unsuccessful_attempts'] = number(metric.get('planning_solver_failures'))
        row.update(planning_solver_calls=np.nan, planning_solver_failures=np.nan, planning_precheck_rejections=np.nan)
        row['command_rate_rms_legacy'] = row['command_rate_rms']
        if include_diagnostics:
            config = json.loads((ROOT / 'config.json').read_text('utf-8'))
            diagnostics, timings = control_diagnostics(file.with_suffix('.mat'), raw, config)
            row.update(diagnostics)
            if any(row.get(f'violation_{name}_maximum', 0.) > 1e-6 for name in ['command_force', 'command_rate']):
                row['safe_success'] = 0
            if timing_groups is not None:
                key = (raw['method'], raw['seed'], raw['mode'])
                group = timing_groups.setdefault(key, {name: [] for name in timings})
                for name, observations in timings.items():
                    group[name].extend(observations)
        row['restricted_completion_seconds'] = min(15, row['completion_seconds']) if np.isfinite(row['completion_seconds']) else 15.
        row['initial_planning_seconds'] = number(metric.get('initial_planning_seconds'))
        events = event_rows(raw)
        for kind, prefix in [('goal_change', 'goal'), ('obstacle_goal_clear', 'obstacle')]:
            selected = [event for event in events if event['event_kind'] == kind and event['observed']]
            observations = np.array([event['rearrival_seconds'] for event in selected])
            row[f'{prefix}_rearrival_seconds_mean'] = observations[np.isfinite(observations)].mean() if np.isfinite(observations).any() else np.nan
            row[f'{prefix}_change_events_observed'] = len(selected)
            row[f'{prefix}_change_events_unreached'] = sum(not event['reached'] for event in selected)
        for category in ['controller_seconds', 'replanning_seconds']:
            for stat in ['median', 'p95', 'maximum']:
                row[f'{category}_{stat}'] = number(metric.get(category, {}).get(stat))
        for constraint, values in metric.get('violations', {}).items():
            row[f'violation_{constraint}_seconds'] = number(values['seconds'])
            row[f'violation_{constraint}_maximum'] = number(values['maximum'])
        result.append(row)
    return result


def write_csv(path, values):
    if not values:
        path.write_text('', 'utf-8')
        return
    fields = list(dict.fromkeys(key for row in values for key in row))
    with path.open('w', newline='', encoding='utf-8-sig') as handle:
        writer = csv.DictWriter(handle, fields)
        writer.writeheader()
        writer.writerows(values)


def strata(values):
    groups = {}
    for row in values:
        key = tuple(row[name] for name in ['method', 'seed', 'family', 'difficulty', 'actuator'])
        groups.setdefault(key, []).append(row)
    result = []
    for key, selected in sorted(groups.items()):
        row = dict(zip(['method', 'seed', 'family', 'difficulty', 'actuator'], key))
        row['runs'] = len(selected)
        row['simulation_failed'] = sum(item['status'] == 'simulation_failed' for item in selected)
        for metric in ['arrival', 'safe_success', 'tight_arrival']:
            row[f'{metric}_rate'] = np.mean([item[metric] for item in selected])
        for metric in ['restricted_completion_seconds', 'position_rms', 'rotation_rms',
                       'actual_force_rms', 'command_rate_rms', 'controller_seconds_median']:
            numbers = np.array([item[metric] for item in selected])
            finite = numbers[np.isfinite(numbers)]
            row[f'{metric}_mean'] = finite.mean() if len(finite) else np.nan
            row[f'{metric}_missing'] = len(numbers) - len(finite)
        result.append(row)
    return result


def pair(values):
    baseline = {(r['id'], r['actuator']): r for r in values if r['method'] == 'baseline'}
    paired = []
    for new in values:
        if new['method'] != 'diffusion':
            continue
        original = baseline.get((new['id'], new['actuator']))
        if original is None:
            continue
        row = {key: new[key] for key in ['id', 'family', 'difficulty', 'actuator', 'seed', 'version']}
        row.update(baseline_status=original['status'], diffusion_status=new['status'])
        for name in ['arrival', 'safe_success', 'tight_arrival', 'restricted_completion_seconds', 'position_rms',
                     'rotation_rms', 'actual_force_rms', 'command_rate_rms', 'controller_seconds_median']:
            row[f'baseline_{name}'] = original[name]
            row[f'diffusion_{name}'] = new[name]
            row[f'difference_{name}'] = new[name] - original[name]
        paired.append(row)
    return paired


def confidence_intervals(paired, seeds, repetitions=10000):
    if not paired:
        return []
    # Resample independent tasks within the 12 registered scenario cells;
    # preserve both actuator profiles together. Resample the five training
    # seeds globally, so all tasks evaluated by a model remain associated.
    conditions = sorted({(r['family'], r['difficulty']) for r in paired})
    task_cells = {cell: sorted({r['id'] for r in paired if (r['family'], r['difficulty']) == cell}) for cell in conditions}
    all_tasks = sorted({r['id'] for r in paired})
    task_index = {task: i for i, task in enumerate(all_tasks)}
    seed_index = {seed: i for i, seed in enumerate(seeds)}
    metrics = ['arrival', 'safe_success', 'tight_arrival', 'restricted_completion_seconds', 'position_rms',
               'rotation_rms', 'actual_force_rms', 'command_rate_rms', 'controller_seconds_median']
    results = []
    for metric in metrics:
        matrix = np.full((len(seeds), len(all_tasks), 2), np.nan)
        for row in paired:
            a = int(row['actuator'] == 'nonideal-force')
            matrix[seed_index[row['seed']], task_index[row['id']], a] = row[f'difference_{metric}']
        complete_tasks = np.all(np.isfinite(matrix), axis=(0, 2))
        if not np.any(complete_tasks):
            continue
        rng = np.random.default_rng(6101)
        samples = np.empty(repetitions)
        for b in range(repetitions):
            selected_tasks = []
            for cell in conditions:
                indices = [task_index[t] for t in task_cells[cell] if complete_tasks[task_index[t]]]
                if indices:
                    selected_tasks.extend(rng.choice(indices, len(indices), replace=True))
            selected_seeds = rng.integers(0, len(seeds), len(seeds))
            samples[b] = matrix[np.ix_(selected_seeds, selected_tasks, [0, 1])].mean()
        low, high = np.quantile(samples, [.025, .975])
        results.append({'metric': metric, 'diffusion_minus_baseline': matrix[:, complete_tasks, :].mean(),
                        'ci_95_low': low, 'ci_95_high': high, 'independent_tasks': int(complete_tasks.sum()),
                        'training_seeds': len(seeds), 'bootstrap_repetitions': repetitions,
                        'missing_task_clusters': int((~complete_tasks).sum())})
    return results


def plots(values):
    outputs = []
    if not values:
        return outputs
    configuration = json.loads((ROOT / 'config.json').read_text('utf-8'))
    fig, ax = plt.subplots(figsize=(10, 4))
    x = np.arange(4)
    for offset, method, label in [(-.18, 'baseline', 'IHSID + force FF LQI + QP'), (.18, 'diffusion', 'Control-increment diffusion')]:
        rates = [np.mean([r['safe_success'] for r in values if r['method'] == method and r['family'] == i]) if any(r['method'] == method and r['family'] == i for r in values) else np.nan for i in range(1, 5)]
        ax.bar(x + offset, rates, .36, label=label)
    ax.set(xticks=x, xticklabels=list(FAMILIES.values()), ylabel='Safe goal success fraction', ylim=(0, 1))
    ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(OUT / 'safe_success.png', dpi=180);plt.close(fig);outputs.append('safe_success.png')
    for metric, label, name in [('position_rms', 'Task position error RMS (m)', 'position_error'),
                                ('rotation_rms', 'Geodesic attitude error RMS (rad)', 'rotation_error'),
                                ('actual_force_rms', 'Actual axial force RMS (N)', 'force_cost'),
                                ('command_rate_rms', 'Command slew RMS (N/s)', 'slew_cost'),
                                ('controller_seconds_p95', 'Episode controller time P95 (s)', 'compute_time')]:
        groups = [[r[metric] for r in values if r['method'] == method and np.isfinite(r[metric])] for method in ['baseline', 'diffusion']]
        if not all(groups):
            continue
        fig, ax = plt.subplots(figsize=(6, 4));ax.boxplot(groups, tick_labels=['Baseline', 'Diffusion'], showfliers=True)
        ax.set_ylabel(label);ax.grid(axis='y', alpha=.2)
        fig.tight_layout();fig.savefig(OUT / f'{name}.png', dpi=180);plt.close(fig);outputs.append(f'{name}.png')
    # Predetermined first task from each family; no outcome-based selection.
    for family in range(1, 5):
        ids = sorted({r['id'] for r in values if r['family'] == family})
        if not ids:
            continue
        for actuator in ['ideal-force', 'nonideal-force']:
            selected = [r for r in values if r['id'] == ids[0] and r['actuator'] == actuator and r['status'] == 'completed']
            if not selected:
                continue
            fig = plt.figure(figsize=(12, 8))
            motion = fig.add_subplot(221, projection='3d')
            position_axis = fig.add_subplot(222)
            rotation_axis = fig.add_subplot(223)
            clearance_axis = fig.add_subplot(224)
            goal_times = []
            plotted_clearance = False
            for row in selected:
                sample = scipy.io.loadmat(ART / 'formal' / row['file'], simplify_cells=True, variable_names=['audit', 'result'])
                audit = sample['audit'];pose = audit['absolute_pose'];time = audit['time']
                label = f"{row['method']} {row['seed']}"
                motion.plot(pose[:, 0], pose[:, 1], pose[:, 2], label=label, alpha=.7)
                position_axis.plot(time, audit['position_error'] * 1000, label=label, alpha=.7)
                rotation_axis.plot(time, np.rad2deg(audit['rotation_error']), label=label, alpha=.7)
                finite = np.isfinite(audit['clearance'])
                if finite.any():
                    clearance_axis.plot(time, np.where(finite, audit['clearance'] * 1000, np.nan), label=label, alpha=.7)
                    plotted_clearance = True
                task = sample['result']['task'];goal_times = np.atleast_1d(task['goal_times'])
                if row['method'] == 'baseline':
                    goals = np.asarray(task['goals']).reshape(6, -1)
                    motion.scatter(goals[0], goals[1], goals[2], marker='x', c='black', label='Task goals')
            for axis in [position_axis, rotation_axis, clearance_axis]:
                axis.set_xlabel('Simulation time (s)');axis.grid(alpha=.2)
                for change in goal_times:
                    if change > 0:
                        axis.axvline(change, color='gray', linestyle=':', linewidth=.8)
            position_axis.set_ylabel('Position error (mm)');position_axis.axhline(configuration['position_tolerance'] * 1000, color='gray', linestyle='--', linewidth=.8)
            rotation_axis.set_ylabel('Geodesic attitude error (deg)');rotation_axis.axhline(configuration['rotation_tolerance_degrees'], color='gray', linestyle='--', linewidth=.8)
            clearance_axis.set_ylabel('Actual minimum clearance (mm)')
            if not plotted_clearance:
                clearance_axis.set_axis_off()
                clearance_axis.text(.5, .5, 'No active obstacle\nClearance is unbounded', transform=clearance_axis.transAxes, ha='center')
            else:
                clearance_axis.axhline(configuration['safe_distance'] * 1000, color='gray', linestyle='--', linewidth=.8)
            motion.set(xlabel='World x (m)', ylabel='World y (m)', zlabel='World z (m)')
            motion.legend(fontsize=7);fig.suptitle(f'{ids[0]}, {actuator}; first registered task in this family\nComplete simulation trajectories: {len(selected)}/6; all outcomes retained in episodes.csv')
            fig.tight_layout();fig.savefig(OUT / f"motion_{family}_{actuator.replace('-', '_')}.png", dpi=180);plt.close(fig);outputs.append(f"motion_{family}_{actuator.replace('-', '_')}.png")
    return outputs


def table(values):
    if not values:
        return '<p>No results in this category yet.</p>'
    columns = list(values[0])
    return '<table><tr>' + ''.join(f'<th>{html.escape(c)}</th>' for c in columns) + '</tr>' + ''.join('<tr>' + ''.join(f'<td>{html.escape(str(r.get(c, "")))}</td>' for c in columns) + '</tr>' for r in values) + '</table>'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--progress', action='store_true', help='Update counts and CSV without expensive final bootstrap/figures')
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    config = json.loads((ROOT / 'config.json').read_text('utf-8'))
    admissions_path = ROOT / 'teacher_source_admissions.json'
    admitted_count = len(json.loads(admissions_path.read_text('utf-8'))['runs']) if admissions_path.exists() else 0
    timing_groups = {}
    values = rows(include_diagnostics=not args.progress, timing_groups=timing_groups);versions = {r['version'] for r in values}
    assert len(versions) <= 1 and (not versions or versions == {config['version']}), 'Experiment versions cannot be pooled.'
    identities = [(r['id'], r['actuator'], r['method'], r['seed']) for r in values]
    assert len(identities) == len(set(identities)), 'Duplicate episode identities.'
    paired = pair(values);summary = []
    for method, seed in [('baseline', 0), *[('diffusion', seed) for seed in config['training_seeds']]]:
        selected = [r for r in values if r['method'] == method and r['seed'] == seed]
        if not selected:
            continue
        summary.append({'method': method, 'seed': seed, 'runs': len(selected), 'completed': sum(r['status'] == 'completed' for r in selected),
                        'simulation_failed': sum(r['status'] == 'simulation_failed' for r in selected),
                        'setup_failed': sum(r['status'] == 'setup_failed' for r in selected),
                        'arrival_rate': np.mean([r['arrival'] for r in selected]), 'safe_success_rate': np.mean([r['safe_success'] for r in selected])})
    ci = [] if args.progress else confidence_intervals(paired, config['training_seeds'])
    write_csv(OUT / 'episodes.csv', values);write_csv(OUT / 'paired.csv', paired);write_csv(OUT / 'summary.csv', summary);write_csv(OUT / 'confidence_intervals.csv', ci)
    write_csv(OUT / 'strata.csv', strata(values))
    write_csv(OUT / 'compute_statistics.csv', pooled_compute_statistics(timing_groups))
    event_values = [event for file in sorted((ART / 'formal').glob('*.json'))
                    for event in event_rows(json.loads(file.read_text('utf-8')))]
    write_csv(OUT / 'events.csv', event_values)
    figures = [] if args.progress else plots(values)
    counts = Counter(r['status'] for r in values)
    complete = (len(values) == 1440 and counts['setup_failed'] == 0
                and sum(r['method'] == 'baseline' for r in values) == 240
                and all(sum(r['method'] == 'diffusion' and r['seed'] == seed for r in values) == 240 for seed in config['training_seeds'])
                and all(r['status'] in ['completed', 'simulation_failed'] for r in values))
    stages = {}
    stage_records = []
    for name, expected in [('teacher', 400), ('development', 144), ('formal', 1440)]:
        stage_values = [json.loads(p.read_text('utf-8')) for p in (ART / name).glob('*_force_*.json')]
        assert all(r['version'] == config['version'] for r in stage_values)
        stages[name] = len(stage_values)
        stage_records.append({'stage': name, 'records': len(stage_values), 'expected': expected, 'statuses': dict(Counter(r['status'] for r in stage_values))})
    models = [json.loads(p.read_text('utf-8')) for p in (ART / 'models').glob('seed_*.json')]
    stages['trained_models'] = sum(bool(m.get('completed', False)) for m in models)
    stage_records.append({'stage': 'trained_models', 'records': stages['trained_models'], 'expected': 5, 'statuses': {}})
    complete = complete and stages['teacher'] == 400 and stages['trained_models'] == 5 and stages['development'] == 144 and (ART / 'freeze.json').exists()
    regression = []
    for profile in ['ideal_force', 'nonideal_force']:
        path = ART / 'regression' / f'strict_qp_{profile}.mat'
        if path.exists():
            verified = scipy.io.loadmat(path, simplify_cells=True, variable_names=['report'])['report']['passed']
            regression.append({'case': profile, 'passed': bool(verified)})
    interface_path = ART / 'validation' / 'interface.json'
    interface = json.loads(interface_path.read_text('utf-8')) if interface_path.exists() else {}
    sources = ''.join(f'<li><a href="{html.escape(url)}">{html.escape(url)}</a></li>' for url in config['references'])
    status = 'Complete formal campaign' if complete else 'Implementation and experiment progress — formal campaign incomplete'
    document = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>第五章规划控制一体化实验</title>
<style>body{{font:16px system-ui;max-width:1200px;margin:40px auto;padding:0 24px;line-height:1.6}}table{{border-collapse:collapse;font-size:13px}}td,th{{border:1px solid #ccc;padding:6px}}img{{max-width:100%}}pre{{white-space:pre-wrap}}.status{{padding:16px;background:#f1f4f8}}</style>
<h1>第五章规划控制一体化：实验报告</h1><p class="status">{status}。正式运行记录 {len(values)}/1440。{html.escape(str(dict(counts)))}</p>
<p>版本：{html.escape(config['version'])}；基线提交：{config['baseline_commit']}。未完成的运行不被称作已经执行的实验。history 内旧版本结果不用于当前训练、统计或完成计数。</p>{table(stage_records)}
<p>{admitted_count} 条较早完成的静态障碍教师运行按 teacher_source_admissions.json 的完整文件哈希登记准入。新旧规划代码的差异是固定边界预检查及恒速度位置表达；障碍速度为零时，新旧位置表达相同，其他共享控制和物理源码必须逐一匹配。全部较早移动障碍记录已隔离并重新采集，未按结果挑选保留者。该准入仅适用于教师数据，不适用于开发和正式对比结果；原始运行没有改写。</p>
<h2>已执行验证</h2>{table(regression)}<p>统一接口验收：{interface.get('passed', False)}。原始验证文件位于 artifacts/validation。开发试跑、教师数据、训练完成与正式留出结果分别计数，不能相互替代。</p>
<h2>方法与边界</h2><p>只比较 IHSID＋力前馈 LQI＋CLF-CBF-QP 与教师数据训练的控制增量扩散方法。新方法输出六维支链力增量，通过含执行器状态的动力学展开得到轨迹；每次去噪后使用带硬约束的预测优化，并在四个候选中选择经后验检查的低代价解。不存在独立生成的跟踪参考轨迹。</p>
<p>预测终端使用可行人工稳态及终端状态等式；实际数值验收容差随配置登记，不放宽碰撞、行程、力或变化率的物理约束。扩散去噪与先验邻近代价构成理想先验目标的计算近似，不把它称为显式求得负对数密度。有限次去噪和优化不宣称已达全局最优。动态障碍与终端条件的组合不宣称具有现成的稳定性或安全证明。QP 回退及上一输入保持均明确记录。</p>
<p>两种方法共用世界坐标、ZYX 旋转、机械及执行器参数、平台与负载碰撞几何和观测信息。仿真环境保留未来事件；控制算法仅获得当前目标和当前可见障碍。运动预测为恒速度。实体碰撞与安全距离不足分别统计；GJK 使用精确凸体支撑映射，未收敛的几何审计不计为安全成功。</p>
<h2>数值和评价协议</h2><p>控制步长 0.02 s，机械内部步长不超过 0.0025 s；15 s 任务期限；60 步预测时域。完整运行满足硬约束且最终目标到达才计为安全成功。到达容差为 5 mm、0.3°、1 mm/s、0.1°/s，连续保持 0.5 s；另报告 1 mm、0.1°结果。误差使用目标位姿和旋转测地距离。完成时间是仿真时间，计算时间单独报告，不解释为墙钟部署响应时间。</p>
<p>基线 IHSID 采用 60 区间隐式 Hermite-Simpson 配点与四固定初值；通用任务关闭插入专属等式。重规划保留当前位置、速度、预计到达时刻及 LQI 状态。目标变化或预计时刻不足 0.5 s 时，重新给定最多六秒并限制在 15 s 截止前留出 0.5 s 保持；时长不足 0.05 s 时记录规划预算耗尽。固定边界的几何不可行性可在求解前排除，四次尝试均保留几何证据和未调用求解器标记。原两阶段任务另按 0.01 s 回归。</p>
<p>输入指令变化率限值 8000 N/s；非理想执行器内部变化率为原模型的 800000 N/s，时间常数 1 ms、输入延迟 1 μs，与控制步长独立。人工稳态偏差权重 100；每次去噪使用 {html.escape(config['projection_solver'])} 的约束优化步骤，最终非线性修正最多 {config['final_projection_iterations']} 次。局部线性约束通过不能替代真实约束验收。四个候选只在真实累加输入的完整动力学展开和终端检查通过后可执行。源码与配置提供全部参数。</p>
<p>预测非线性搜索在每个控制区间端点施加物理约束，以降低冗余求解规模；执行前单独检查全部内部积分点的行程、速度、加速度、奇异性、实际力及碰撞裕度。内部点失败的候选被排除，即使求解器声明成功也不执行。此网格验收不等同于连续时间安全证明。</p>
<p>原连续 LQI 直接采样到 50 Hz 会产生不稳定极点。通用任务据此采用 50 Hz 离散增广模型上的 LQI，保留原二次型权重、积分泄漏及反馈力限幅，并使用共享的位姿/速度观测；重规划不重置积分状态。该离散实现调整在独立开发任务上验证，不根据正式留出结果选取。</p>
<h2>正式结果</h2>{table(summary)}<h2>配对差异和置信区间</h2>{table(ci)}
<p>差异定义为新方法减基线。成功率越高越好，时间、误差和控制代价越低越好。成功判据的失败运行保留在分母中；连续指标缺测的任务簇数量单列。完成时间另用 15 s 截断值，避免只统计成功者。按场景难度分层重采样完整任务，同时保留两种执行器条件，并在五个训练种子间重采样。确定性基线不作为五个独立样本重复计数。</p>
<p>目标变化后重新到达时间不含初始到达阶段。events.csv 逐项保存变化事件、是否观测到、是否重新到达及观测随访时间，未重新到达不从事件记录中删除。规划前的几何拒绝与实际求解器失败分别计数。完整控制计算包含去噪、预测优化、候选检查和必要的原生首次编译；计时从共享观测与执行器状态更新之后开始，结束于力指令返回。</p>
<p>compute_statistics.csv 按方法、训练种子和执行器配置汇总所有实际控制调用及规划调用的中位数、P95 和最大耗时，不将各运行的 P95 再当作总体 P95。指令变化率按每个已执行区间的指令减上一拍指令计算，包含首拍，按区间时长加权；仿真终止时未执行的输出仅保留计算耗时。原存储指标另列 command_rate_rms_legacy，指令力和变化率约束分别列出违反时间与最大违反量。</p>
{''.join(f'<p><img src="{name}" alt="{name}"></p>' for name in figures)}
<h2>可复算文件</h2><ul><li><a href="episodes.csv">全部运行</a></li><li><a href="paired.csv">任务配对</a></li><li><a href="summary.csv">按训练种子汇总</a></li><li><a href="strata.csv">工况、难度、执行器及种子分层</a></li><li><a href="events.csv">变化事件与重新到达</a></li><li><a href="compute_statistics.csv">全部调用的计算耗时统计</a></li><li><a href="confidence_intervals.csv">置信区间</a></li></ul>
<p>原始 MAT 保存机械状态、指令、Simscape 支链力、事件、约束审计、预测候选及求解诊断。控制器执行器状态是用已知模型递推的状态估计；不把它称为独立传感器测量。几何障碍用于碰撞检查和避障约束，当前工程没有障碍接触力学。仅以这些仿真判断实体几何交叠，不能验证真实碰撞后的行为。</p>
<p>教师失败记录不删除；仅成功且物理有效的完整运行提取训练窗口。训练/验证以完整任务分离，归一化只用训练集；窗口跨未知事件时排除。五个种子分别根据验证损失选取检查点，全部参加正式实验。每次重试有独立任务、来源、进程及日志身份；每秒增量保存控制记录，异常仿真尽可能保留 Simscape 的部分输出。进程被强制终止可能缺少最后一秒和未返回的连续输出，此限制单独审计。</p>
<h2>文献依据</h2><ul>{sources}</ul><p>上述工况数值是预登记工程设定，不冒充文献统一标准。MPD 的批量规划成功率不能直接等同闭环任务成功率。结果仅支持本 Simscape 工程、这些参数和留出任务，不覆盖实体平台试验。</p>
<h2>冻结配置</h2><pre>{html.escape(json.dumps(config, ensure_ascii=False, indent=2))}</pre></html>'''
    (OUT / 'index.html').write_text(document, 'utf-8')
    (OUT / 'completion.json').write_text(json.dumps({'complete': complete, 'formal_records': len(values), 'statuses': dict(counts), 'stages': stages}, indent=2), 'utf-8')
    print(f'{status}: {len(values)}/1440 records. Report: {OUT / "index.html"}')


if __name__ == '__main__':
    main()
