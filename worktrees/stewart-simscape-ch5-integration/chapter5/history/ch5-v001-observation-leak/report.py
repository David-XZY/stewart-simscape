"""Recomputable paired, task-clustered, training-seed-stratified summaries."""
from __future__ import annotations

import csv
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


def rows():
    result = []
    for file in sorted((ART / 'formal').glob('*.json')):
        raw = json.loads(file.read_text('utf-8'))
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
                     'planning_attempts', 'planning_solver_failures', 'planning_rejections', 'predictive_solves', 'predictive_solver_failures']:
            row[name] = number(metric.get(name))
        row['restricted_completion_seconds'] = min(15, row['completion_seconds']) if np.isfinite(row['completion_seconds']) else 15.
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
        selected = [r for r in values if r['id'] == ids[0] and r['actuator'] == 'ideal-force' and r['status'] == 'completed']
        fig, ax = plt.subplots(figsize=(6, 5))
        for row in selected:
            sample = scipy.io.loadmat(ART / 'formal' / row['file'], simplify_cells=True, variable_names=['audit'])
            pose = sample['audit']['absolute_pose']
            ax.plot(pose[:, 0], pose[:, 1], label=f"{row['method']} {row['seed']}", alpha=.7)
        ax.set(xlabel='World x (m)', ylabel='World y (m)', title=f'{ids[0]}, ideal force');ax.axis('equal');ax.legend(fontsize=7)
        fig.tight_layout();fig.savefig(OUT / f'motion_{family}.png', dpi=180);plt.close(fig);outputs.append(f'motion_{family}.png')
    return outputs


def table(values):
    if not values:
        return '<p>No results in this category yet.</p>'
    columns = list(values[0])
    return '<table><tr>' + ''.join(f'<th>{html.escape(c)}</th>' for c in columns) + '</tr>' + ''.join('<tr>' + ''.join(f'<td>{html.escape(str(r.get(c, "")))}</td>' for c in columns) + '</tr>' for r in values) + '</table>'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    config = json.loads((ROOT / 'config.json').read_text('utf-8'))
    values = rows();versions = {r['version'] for r in values}
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
    ci = confidence_intervals(paired, config['training_seeds'])
    write_csv(OUT / 'episodes.csv', values);write_csv(OUT / 'paired.csv', paired);write_csv(OUT / 'summary.csv', summary);write_csv(OUT / 'confidence_intervals.csv', ci)
    figures = plots(values)
    counts = Counter(r['status'] for r in values)
    complete = len(values) == 1440 and counts['setup_failed'] == 0 and all(r['status'] in ['completed', 'simulation_failed'] for r in values)
    stages = {}
    for name in ['validation', 'regression', 'teacher', 'models', 'development', 'formal']:
        stages[name] = len(list((ART / name).glob('*.mat')))
    regression = []
    for profile in ['ideal_force', 'nonideal_force']:
        path = ART / 'regression' / f'strict_qp_{profile}.txt'
        if path.exists():
            regression.append({'case': profile, 'passed': 'passed: 1' in path.read_text('utf-8', errors='replace')})
    interface_path = ART / 'validation' / 'interface.json'
    interface = json.loads(interface_path.read_text('utf-8')) if interface_path.exists() else {}
    sources = ''.join(f'<li><a href="{html.escape(url)}">{html.escape(url)}</a></li>' for url in config['references'])
    status = 'Complete formal campaign' if complete else 'Implementation and experiment progress — formal campaign incomplete'
    document = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>第五章规划控制一体化实验</title>
<style>body{{font:16px system-ui;max-width:1200px;margin:40px auto;padding:0 24px;line-height:1.6}}table{{border-collapse:collapse;font-size:13px}}td,th{{border:1px solid #ccc;padding:6px}}img{{max-width:100%}}pre{{white-space:pre-wrap}}.status{{padding:16px;background:#f1f4f8}}</style>
<h1>第五章规划控制一体化：实验报告</h1><p class="status">{status}。正式运行记录 {len(values)}/1440。{html.escape(str(dict(counts)))}</p>
<p>版本：{html.escape(config['version'])}；基线提交：{config['baseline_commit']}。阶段产物数量：{html.escape(str(stages))}。未完成的运行不被称作已经执行的实验。</p>
<h2>已执行验证</h2>{table(regression)}<p>统一接口验收：{interface.get('passed', False)}。原始验证文件位于 artifacts/validation。开发试跑、教师数据、训练完成与正式留出结果分别计数，不能相互替代。</p>
<h2>方法与边界</h2><p>只比较 IHSID＋力前馈 LQI＋CLF-CBF-QP 与教师数据训练的控制增量扩散方法。新方法输出六维支链力增量，通过含执行器状态的动力学展开得到轨迹；每次去噪后使用带硬约束的预测优化，并在四个候选中选择经后验检查的低代价解。不存在独立生成的跟踪参考轨迹。</p>
<p>预测终端使用可行人工稳态及终端状态等式。去噪、有限迭代非线性优化与动态障碍组合的方法不宣称具有已证明的全局最优性、稳定性或安全保证。QP 回退及上一输入保持均明确记录。DPCC 的约束投影思想、MPCT 的人工稳态思想是方法依据，并非对 Stewart 组合方法的现成证明。</p>
<p>两种方法共用世界坐标、ZYX 旋转、机械及执行器参数、平台与负载碰撞几何和观测信息。仿真环境保留未来事件；控制算法仅获得当前目标和当前可见障碍。运动预测为恒速度。实体碰撞与安全距离不足分别统计；GJK 使用精确凸体支撑映射，未收敛的几何审计不计为安全成功。</p>
<h2>数值和评价协议</h2><p>控制步长 0.02 s，机械内部步长不超过 0.0025 s；15 s 任务期限；60 步预测时域。完整运行满足硬约束且最终目标到达才计为安全成功。到达容差为 5 mm、0.3°、1 mm/s、0.1°/s，连续保持 0.5 s；另报告 1 mm、0.1°结果。误差使用目标位姿和旋转测地距离。完成时间是仿真时间，计算时间单独报告，不解释为墙钟部署响应时间。</p>
<p>基线 IHSID 采用 60 区间隐式 Hermite-Simpson 配点与四固定初值；通用任务关闭插入专属等式。重规划保留当前位置、速度及 LQI 状态。原两阶段任务另按 0.01 s 回归。输入指令变化率限值 8000 N/s。人工稳态偏差权重 100；预测投影每次去噪至多 12 次 IPOPT 迭代，只有后验通过的候选可执行。全部实现参数随配置、源代码和冻结哈希追溯。</p>
<p>原连续 LQI 直接采样到 50 Hz 会产生不稳定极点。通用任务据此采用 50 Hz 离散增广模型上的 LQI，保留原二次型权重、积分泄漏及反馈力限幅，并使用共享的位姿/速度观测；重规划不重置积分状态。该离散实现调整在独立开发任务上验证，不根据正式留出结果选取。</p>
<h2>正式结果</h2>{table(summary)}<h2>配对差异和置信区间</h2>{table(ci)}
<p>差异定义为新方法减基线。成功率越高越好，时间、误差和控制代价越低越好。成功判据的失败运行保留在分母中；连续指标缺测的任务簇数量单列。完成时间另用 15 s 截断值，避免只统计成功者。按场景难度分层重采样完整任务，同时保留两种执行器条件，并在五个训练种子间重采样。确定性基线不作为五个独立样本重复计数。</p>
{''.join(f'<p><img src="{name}" alt="{name}"></p>' for name in figures)}
<h2>可复算文件</h2><ul><li><a href="episodes.csv">全部运行</a></li><li><a href="paired.csv">任务配对</a></li><li><a href="summary.csv">按训练种子汇总</a></li><li><a href="confidence_intervals.csv">置信区间</a></li></ul>
<p>原始 MAT 保存状态、指令、实测支链力、事件、约束审计、预测候选及求解诊断。教师失败记录不删除；仅成功且物理有效的完整运行提取训练窗口。训练/验证以完整任务分离，归一化只用训练集；窗口跨未知事件时排除。五个种子分别根据验证损失选取检查点，全部参加正式实验。</p>
<h2>文献依据</h2><ul>{sources}</ul><p>上述工况数值是预登记工程设定，不冒充文献统一标准。MPD 的批量规划成功率不能直接等同闭环任务成功率。结果仅支持本 Simscape 工程、这些参数和留出任务，不覆盖实体平台试验。</p>
<h2>冻结配置</h2><pre>{html.escape(json.dumps(config, ensure_ascii=False, indent=2))}</pre></html>'''
    (OUT / 'index.html').write_text(document, 'utf-8')
    (OUT / 'completion.json').write_text(json.dumps({'complete': complete, 'formal_records': len(values), 'statuses': dict(counts), 'stages': stages}, indent=2), 'utf-8')
    print(f'{status}: {len(values)}/1440 records. Report: {OUT / "index.html"}')


if __name__ == '__main__':
    main()
