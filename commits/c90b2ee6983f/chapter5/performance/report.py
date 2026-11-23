"""Recompute the independent performance probe tables and figures from raw results."""
import csv
import hashlib
import html
import json
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / 'chapter5/artifacts/performance_probe'
OUT = DATA / 'report'
OUT.mkdir(exist_ok=True)


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def table(headers, rows):
    return '<table><tr>' + ''.join(f'<th>{html.escape(str(v))}</th>' for v in headers) + '</tr>' + ''.join(
        '<tr>' + ''.join(f'<td>{html.escape(str(v))}</td>' for v in row) + '</tr>' for row in rows) + '</table>'


def csv_rows(name, rows):
    if not rows:
        return
    with (OUT / name).open('w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)


bench = read(DATA / 'benchmark.json')
budget = read(DATA / 'budget_v4/study.json') if (DATA / 'budget_v4/study.json').exists() else None
planning = read(DATA / 'planning_paired/study.json') if (DATA / 'planning_paired/study.json').exists() else None
legacy = read(DATA / 'closed_loop/001_ideal_force.json')
prior = read(DATA / 'continued_v2/001_ideal_force.json')
corrected_files = sorted((DATA / 'corrected_closed_loop').glob('*_ideal_force.json'))
corrected = [read(p) for p in corrected_files]
tail_files = sorted((DATA / 'tail_closed_loop').glob('*_ideal_force.json'))
tail = [read(p) for p in tail_files]
timing = []
for r in bench['rows']:
    timing.append(dict(version='initial_reduction_v1', mode=r['mode'], budget=r['label'], sample=r['sample'],
        seconds=r['seconds'], deadline_met=r['deadline_met'], fallback=r['fallback'],
        initial_position_error=r['position_error'], terminal_position_error=r['terminal_position_error']))
if budget:
    for r in budget['rows']:
        timing.append(dict(version='braking_budget_v4', mode=r['mode'], budget=f"NLP{r['polish_iterations']}", sample=r['sample'],
            seconds=r['seconds'], deadline_met=r['deadline_met'], fallback=r['fallback'],
            initial_position_error=r['initial_position_error'], terminal_position_error=r['terminal_position_error']))
csv_rows('control_timing.csv', timing)

parts = ['<h1>第五章速度改进开发验证</h1>', f'<p>生成时间：{datetime.now().astimezone().isoformat(timespec="seconds")}</p>',
    '<p>本报告为独立开发试验，不计入正式1440次实验。沿用原物理约束、执行器动力学、五个已训练模型中的5101模型；轨迹仍由支链力增量的动力学展开得到。没有放宽终端条件或把预测终点当成实测到达。</p>',
    '<p>10 Hz优化的预算为100毫秒，50 Hz控制执行的预算为20毫秒。仿真同步等待求解，因此运动时间不包含计算造成的真实部署延迟。编译及模型构建与逐次控制计时分别记录；首次调用可能另有惰性初始化。</p>',
    '<h2>减少重复优化的结果</h2>']
parts.append(table(['版本','执行器','预算','样本','耗时 ms','100 ms内','预测终点误差 mm','回退'], [
    [r['version'],r['mode'],r['budget'],r['sample'],f"{r['seconds']*1000:.2f}",r['deadline_met'],
     f"{r['terminal_position_error']*1000:.3f}",r['fallback']] for r in timing]))
parts.append('<p>v1移除每次去噪的重复投影，4个候选排序后只校正1个，限制SQP及最终NLP次数。v4在旧输入序列失去终端可行性时重建原有动力学制动初值，并限制未完成迭代的物理不等式。少量迭代保留的可行静态候选不构成到达目标。</p>')
if budget:
    warmed = [r for r in budget['rows'] if r['sample']==2]
    fig, ax = plt.subplots(figsize=(7,3.6));labels=[f"{r['mode'].split('-')[0]} / {r['polish_iterations']} NLP" for r in warmed]
    ax.bar(labels,[r['seconds']*1000 for r in warmed],color='#387cb9');ax.axhline(100,color='#c54545',linestyle='--',label='100 ms budget')
    ax.set_ylabel('Complete call time (ms)');ax.set_yscale('log');ax.tick_params(axis='x',rotation=30);ax.legend();fig.tight_layout()
    fig.savefig(OUT/'latency.png',dpi=170);plt.close(fig);parts.append('<img src="latency.png" alt="同输入扰动下优化预算与耗时">')

parts.append('<h2>Simscape完整闭环及失败审计</h2>')
loops = [('unfinished_v2',prior),('guarded_v3',legacy)] + [('corrected_v7',r) for r in corrected] + [('tail_v9',r) for r in tail]
loop_csv = []
for version,r in loops:
    m=r['metrics'];refresh=np.asarray(r['refresh_seconds']);hold=np.asarray(r['execution_check_seconds'])
    loop_csv.append(dict(version=version,task=r['task'],mode=r['mode'],wall_seconds=r['wall_seconds'],
        simulated_seconds=m['simulated_end_seconds'],arrival=m['arrival'],safe_success=m['safe_success'],
        physical_constraints_valid=m['all_constraints_valid'],final_position_error_m=m['final_position'],
        fallback_count=m['fallback_count'],control_samples=m['control_samples'],refresh_calls=len(refresh),
        refresh_median_seconds=float(np.median(refresh)) if len(refresh) else None,
        refresh_p95_seconds=float(np.percentile(refresh,95)) if len(refresh) else None,
        refresh_maximum_seconds=float(np.max(refresh)) if len(refresh) else None,
        refresh_deadline_misses=r['refresh_deadline_misses'],hold_deadline_misses=r['execution_deadline_misses'],
        simulation_error=r.get('simulation_error','')))
csv_rows('closed_loop.csv',loop_csv)
parts.append(table(['版本','任务','仿真秒','墙钟秒','到达','全程约束','末端误差 mm','回退/控制拍','规划调用','规划中位 ms','100 ms超时'],[
    [r['version'],r['task'],f"{r['simulated_seconds']:.3f}",f"{r['wall_seconds']:.2f}",r['arrival'],r['physical_constraints_valid'],
     f"{r['final_position_error_m']*1000:.3f}",f"{r['fallback_count']}/{r['control_samples']}",r['refresh_calls'],
     f"{r['refresh_median_seconds']*1000:.2f}" if r['refresh_median_seconds'] is not None else '',r['refresh_deadline_misses']] for r in loop_csv]))
parts.append('<p>v3完整15秒试验失败：748/751拍回退，随后实际支链加速度、速度、行程及奇异性约束违反并发生仿真发散。物理范围之外的巨大误差只是失败诊断，不能解释为平台的有效运动范围。不得将输入未超限或初始候选可行称为全程安全。</p><p>v7保留失败时的10 Hz刷新时钟，重建失效的初值，采用30次最终NLP迭代。固定目标完整运行仍未到达并有短时加速度违反。v9保留下一规划力并重建后续制动序列，仍须通过原完整检查；两个任务均因物理违反提前停止。任务3的目标切换事件在4秒，而运行提前结束，不能评价目标切换响应。</p><p>v5入口参数赋值顺序错误、v6未刷新缓存入口、v7任务3读取开发中语法错误导致零控制样本，全部原始错误日志保留，单独计为入口失败，不能当作控制性能数据。语法增补已隔离到独立文件。</p>')
parts.append(table(['版本','约束','违反累计秒','最大违反量（各自SI单位）'],[
    [version,name,f"{v['seconds']:.6f}",f"{v['maximum']:.8g}"] for version,r in loops for name,v in r['metrics']['violations'].items()]))
if corrected or tail:
    fig,axs=plt.subplots(2,1,figsize=(7,5),sharex=True)
    for path in [*sorted((DATA/'corrected_closed_loop').glob('*_ideal_force.mat')),*sorted((DATA/'tail_closed_loop').glob('*_ideal_force.mat'))]:
        if not path.with_suffix('.json').exists():
            continue
        raw=loadmat(path,simplify_cells=True);audit=raw['audit'];records=raw['records'];records=[records] if isinstance(records,dict) else records
        axs[0].plot(audit['time'],np.asarray(audit['position_error'])*1000,label=f"{path.parent.name} / task {raw['row']['task']}")
        axs[1].plot([r['time'] for r in records],[r['controller_seconds']*1000 for r in records])
    axs[0].set_ylabel('Position error (mm)');axs[0].legend();axs[1].set_ylabel('Control call (ms)');axs[1].set_xlabel('Simulation time (s)');axs[1].axhline(100,ls='--',color='r');fig.tight_layout();fig.savefig(OUT/'closed_loop.png',dpi=170);plt.close(fig)
    parts.append('<img src="closed_loop.png" alt="修正版实测误差与控制耗时">')

parts.append('<h2>基线规划与密集检查</h2>')
if planning:
    rows=planning['dense_rows'];csv_rows('dense_check.csv',rows)
    parts.append(table(['样本','原检查秒','批量检查秒','最低裕度差','通过'],[
        [r['sample'],f"{r['original_seconds']:.6f}",f"{r['batched_seconds']:.6f}",r['minimum_margin_error'],r['accepted']] for r in rows]))
    prows=planning['planning_rows'];parts.append(table(['初值','总调用秒','构建秒','通过','四次迭代数'],[
        [r['label'],f"{r['seconds']:.4f}",f"{r['plan']['build_seconds']:.4f}",r['accepted'],
         ','.join(str(a.get('iterations')) for a in r['plan']['attempts'])] for r in prows]))
    csv_rows('planning.csv',[dict(label=r['label'],seconds=r['seconds'],accepted=r['accepted'],build_seconds=r['plan']['build_seconds']) for r in prows])
    parts.append(f"<p>比较使用完全相同的当前位姿、速度、目标、障碍观测及剩余时长。参考重建的全网格最大差为 {planning['reference_error']:.3g}；四组初值及1000次迭代上限保留。快速不可行拒绝不当作成功规划。该冷/热配对各一次，不能作为总体统计结论。</p>")
    parts.append('<p>空障碍索引的缓存键分别为 '+html.escape(str(planning['empty_index_cache_keys']))+'。v4预构建使用了不同形状的空数组，首次控制另有优化器构建；已修复预构建索引，表格仍保留原首次耗时。</p>')
else:
    parts.append('<p>同状态冷启动/热启动配对试验尚未完成，不报告提速比例。</p>')

parts.append('<h2>追溯与边界</h2><p>所有尝试，包括v2未完成迭代导致NaN、v3物理失败、停止的重复任务，都保留日志及可用原始数据。版本结果分开保存。同任务跨版本重复不是独立样本。正式数据仍为0/1440；本报告不比较两方法的正式胜率，不选择更优训练种子。当前验证范围不能推出移动障碍安全或整个组合方法的稳定性。</p>')
parts.append('<ul>'+''.join(f'<li><a href="../{p}">{p}</a></li>' for p in ['benchmark.mat','budget_v4/study.mat','closed_loop/001_ideal_force.mat','corrected_closed_loop/summary.mat','tail_closed_loop/summary.mat','planning_paired/study.mat','guarded_interruption.json','preservation_audit.json','corrected_closed_loop/003_failure.json'])+'</ul>')
manifest=[]
for path in sorted(DATA.rglob('*')):
    if path.is_file() and path.suffix in {'.mat','.json','.m','.log'} and 'report' not in path.relative_to(DATA).parts and 'journal' not in str(path):
        manifest.append(dict(path=str(path.relative_to(ROOT)),bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
(OUT/'artifact_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
style='body{max-width:1100px;margin:30px auto;font:16px/1.7 sans-serif;color:#243344}table{border-collapse:collapse;font-size:13px;width:100%;margin:18px 0}td,th{border:1px solid #ccd6df;padding:7px;text-align:left}th{background:#e8f0f7}img{max-width:850px;width:100%}h1,h2{line-height:1.3}'
(OUT/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>第五章速度开发验证</title><style>'+style+'</style><body>'+''.join(parts)+'</body></html>',encoding='utf-8')
print(OUT/'index.html')
