"""Recompute the development report from preserved result states and raw audits."""
from pathlib import Path
import csv, datetime, hashlib, html, json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
BASE=ROOT/'chapter5/artifacts/control_force_v62'
OUT=BASE/'implementation_20261013'

def read(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
def number(value,scale=1):return '—' if value is None else f'{value*scale:.3f}'

def main():
    OUT.mkdir(exist_ok=True,parents=True);timestamp=datetime.datetime.now(datetime.timezone.utc).isoformat()
    offline=read(BASE/'online_v80/offline_raw_v81.json');assert offline['completed'] and len(offline['episodes'])==20
    raw={(r['version'],r['episode']):r for r in offline['episodes']};rows=[]
    for version in [77,78,79,80,81]:
        report=read(BASE/f'full_v{version}/summary.json');assert report['completed'] and len(report['rows'])==4
        for row in report['rows']:
            m=row['metrics'];name=f"{row['task']:03d}_{row['mode'].replace('-','_')}.mat";a=raw[(version,name)]
            rows.append(dict(version=version,task=row['task'],mode=row['mode'],arrival=m['arrival'],tight_arrival=m['tight_arrival'],
                all_constraints_valid=m['all_constraints_valid'],final_position_mm=m['final_position']*1000,
                completion_seconds=m['completion_seconds'],rearrival_seconds=m['rearrival_seconds'][-1] if isinstance(m['rearrival_seconds'],list) else m['rearrival_seconds'],
                fallback=m['fallback_count'],wall_seconds=row['wall_seconds'],refresh_median_ms=row['refresh_stats']['median']*1000,
                refresh_p95_ms=row['refresh_stats']['p95']*1000,refresh_maximum_ms=row['refresh_stats']['maximum']*1000,
                raw_generated=a['generated'],raw_accepted=a['accepted'],source=f'full_v{version}/{name}'))
    with (OUT/'development_results.csv').open('w',newline='',encoding='utf-8-sig') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    models={str(v):read(BASE/f'raw_v{v}/seed_5101/metadata.json') for v in [78,80]}
    fixed={str(v):read(BASE/f'raw_v{v}/raw_report.json') for v in [78,80]}
    old=read(BASE/'raw_v76/source_manifest.json');old_source_changes=[name for name,digest in old.items() if not (ROOT/name).is_file() or sha(ROOT/name)!=digest]
    assert not old_source_changes,f'Historical frozen source changed: {old_source_changes}'
    sources=set(old)
    for p in (ROOT/'chapter5/reduced').rglob('*'):
        if p.is_file() and p.suffix in ['.m','.py','.txt'] and '__pycache__' not in str(p):sources.add(str(p.relative_to(ROOT)))
    manifest={name:sha(ROOT/name) for name in sorted(sources)};write(OUT/'source_manifest.json',manifest)
    artifacts=[]
    for name in ['online_v77','online_v79','online_v80','online_v81','raw_v78','raw_v80','full_v77','full_v78','full_v79','full_v80','full_v81','nonlinear_v80']:
        for p in (BASE/name).rglob('*'):
            if p.is_file() and p.suffix in ['.json','.mat','.pt','.csv','.c','.dll']:artifacts.append(p)
    write(OUT/'artifact_manifest.json',{str(p.relative_to(ROOT)):sha(p) for p in sorted(artifacts)})
    pilot=BASE/'teacher_pilot_v82';pilot_summary=read(pilot/'summary.json') if (pilot/'summary.json').exists() else dict(completed=False,rows=[])
    pilot_active=read(pilot/'active.json') if (pilot/'active.json').exists() else None
    memory=read(BASE/'online_v81/os_memory.json');worker=read(ROOT/'chapter5/artifacts/performance_probe/worker_status.json')
    baseline_source=read(BASE/'full_v76/summary.json')['rows'][0]
    selected=next(r for r in rows if r['version']==79 and r['task']==4)
    summary=dict(timestamp_utc=timestamp,formal=False,formal_runs=0,expected_formal_runs=1440,
        completed_full_development_runs=20,completed_short_runs=3,independent_samples=False,
        selected_development_version=79,selection_reason='Preserves moving-obstacle arrival, zero fallback and lower final error than the nonlinear-training pilot; no test tasks used.',
        selected_moving=selected,previous_moving_refresh_median_ms=float(np.median(baseline_source['refresh_seconds'])*1000),
        fixed_raw_20_step_accepted=87,fixed_raw_ten_step_accepted=fixed['78']['full_accepted'],fixed_raw_nonlinear_task_accepted=fixed['80']['full_accepted'],
        raw_candidates_reaudited=len(offline['rows']),raw_arrays_define_denominator=True,
        teacher_pilot_expected_runs=18,teacher_pilot_completed_states=len(pilot_summary['rows']),teacher_pilot_completed=pilot_summary['completed'],
        teacher_pilot_admitted=sum(r['admitted'] for r in pilot_summary['rows']),teacher_pilot_active=pilot_active,
        teacher_pilot_model_trained=False,training_seed_count=1,training_seeds=[5101],
        nonlinear_gradient_parity=read(BASE/'nonlinear_v80/parity.json')['accepted'],
        preserved_historical_source_count=len(old),source_manifest_count=len(manifest),worker=worker,memory=memory)
    write(OUT/'summary.json',summary)
    moving=[r for r in rows if r['task']==4];fig,axes=plt.subplots(1,2,figsize=(10,3.6),layout='constrained')
    labels=[f"V{r['version']}" for r in moving];axes[0].bar(labels,[r['refresh_median_ms'] for r in moving],color='#3b7285')
    axes[0].axhline(100,color='#b84b38',linestyle='--',label='100 ms');axes[0].legend();axes[0].set_ylabel('Refresh median (ms)')
    axes[1].bar(labels,[r['final_position_mm'] for r in moving],color=['#b84b38' if not r['arrival'] else '#3b7285' for r in moving]);axes[1].set_ylabel('Final position error (mm)')
    fig.savefig(OUT/'moving_development.png',dpi=160);plt.close(fig)
    table=''.join('<tr>'+''.join(f'<td>{html.escape(str(v))}</td>' for v in [
        r['version'],r['task'],r['mode'],r['arrival'],r['tight_arrival'],r['all_constraints_valid'],
        number(r['final_position_mm']),number(r['wall_seconds']),number(r['refresh_median_ms']),number(r['refresh_p95_ms']),
        r['fallback'],f"{r['raw_accepted']}/{r['raw_generated']}"] )+'</tr>' for r in rows)
    active_text='未开始' if pilot_active is None else f"{pilot_active['task']} / {pilot_active['mode']} / {pilot_active['status']}"
    teacher_table=''.join('<tr>'+''.join(f'<td>{html.escape(str(v))}</td>' for v in [
        r['task'],r['mode'],r['status'],r['admitted'],r.get('metrics',{}).get('fallback_count','—'),
        number(r.get('metrics',{}).get('minimum_clearance'),1000)])+'</tr>' for r in pilot_summary['rows'])
    description=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>第五章实施与开发验证 · 2026-10-13</title>
<style>body{{font:16px/1.65 'Microsoft YaHei',sans-serif;max-width:1250px;margin:32px auto;padding:0 20px;color:#25313b}}table{{border-collapse:collapse;font-size:13px;width:100%}}td,th{{border:1px solid #d9e1e6;padding:7px}}th{{background:#eaf0f4}}a{{color:#18657c}}img{{max-width:100%}}code{{background:#edf1f4;padding:2px 5px}}.note{{background:#f2f5f7;padding:14px}}</style>
<h1>第五章实施与开发验证</h1><p>快照时间：{timestamp}。本轮完成 20 次完整 15 s 开发运行和 3 次短运行。重复任务及算法版本不构成新增独立样本。正式实验仍为 0/1440。</p>
<p class="note">建议保留 V79 作为当前开发起点：移动障碍到达、全程满足数值检查、零回退，刷新中位数从 449.419 ms 降至 {selected['refresh_median_ms']:.3f} ms。最终位置误差 {selected['final_position_mm']:.3f} mm；未通过严格 1 mm 判据，也未达到完整控制 P95≤100 ms。仿真时间与墙钟耗时分别列出。</p>
<h2>实现与证据</h2><ul><li>缓存执行器瞬态检查的映射函数；共享 QP 准备；候选先生成两个，不成功时再扩展。执行前保留完整内部网格、碰撞、执行器瞬态、终端区域和下降检查。</li>
<li>诊断开关与控制选择分离，4 个任务/执行器条件的输入差为零。注入拒绝后清除接受轨迹，记录安全 QP 回退；回退不作为安全保证。</li>
<li>首个可行候选策略 V77/V78 在移动障碍上退化至约 38 mm，已保留并排除。V79 比较生成候选的优化任务代价，并仅检查可能更优的原始候选，恢复到达。</li>
<li>V78 对实际 10 步采样过程训练并蒸馏，MATLAB/Python 使用同一时间网格。同条件、同噪声 96 个候选：20 步原模型 87 个通过；10 步训练模型 88 个通过。</li>
<li>V80 的任务梯度经真实执行器增广动力学和反馈增量映射反传，阶段 A/B 及 12 个梯度坐标通过核对。约束训练仍使用教师局部代理，最终接受仍由非线性完整检查决定。控制代价使用相对辅助稳态力 u−u_s。</li>
<li>V80 原始候选仍为 88/96，移动任务误差增至约 1.48 mm，暂不作为质量提升采用。V81 的编译递推改写输入输出一致，但完整运行未获得显著速度收益。</li></ul>
<h2>完整开发运行</h2><p>任务 1 为固定目标，任务 3 在 8 s 切换目标，任务 4 为连续移动障碍。单位：位置 mm、墙钟 s、刷新 ms。原始候选分母来自实际保存的生成数组，已在执行后完整检查 {len(offline['rows'])} 个候选；不把 QP 或尾段修复结果计为原始生成成功。</p>
<img src="moving_development.png" alt="移动障碍开发版本耗时和误差"><table><tr><th>版本</th><th>任务</th><th>执行器</th><th>到达</th><th>严格到达</th><th>完整约束</th><th>最终误差</th><th>墙钟</th><th>刷新中位数</th><th>刷新P95</th><th>回退</th><th>原始通过/生成</th></tr>{table}</table>
<h2>动态教师数据</h2><p>已冻结 6 个训练任务、3 个验证任务，覆盖横穿、接近、连续转向；每个任务两种执行器，共 18 次。当前有 {len(pilot_summary['rows'])}/18 个结果状态，其中 {summary['teacher_pilot_admitted']} 个可用于训练；当前任务：{html.escape(active_text)}。这是采集快照，尚未据此训练动态模型。</p>
<p>教师保持 IHSID＋力前馈 LQI＋CLF-CBF-QP。窗口每五步提取，标签为原始最终 QP 力增量，不修尾、不补齐；窗口避开未预告的障碍出现事件。训练/验证按完整任务隔离，归一化只用训练任务。失败完整保留并从训练窗口中排除。</p>
<table><tr><th>任务</th><th>执行器</th><th>结果状态</th><th>可训练</th><th>回退次数</th><th>最小净空 mm</th></tr>{teacher_table}</table>
<p>当前采集配置保持冻结。已出现的连续转向失败包含 QP 不可行和安全距离不足；故障记录用于后续排查，任何算法修复另开版本，不混入当前教师数据。实体碰撞与安全距离不足在原始指标中分别记录。</p>
<h2>运行、内存与失败保留</h2><p>本轮只因旧 MATLAB 进程已退出而新建一次会话，此后均使用 PID {memory['pid']}。内存快照：私有内存 {memory['private_bytes']/2**30:.2f} GiB，系统可用物理内存 {memory['free_physical_bytes']/2**30:.2f} GiB；本轮未出现内存不足。</p>
<p>首次离线审计暴露了空数组候选计数问题，原始数组及失败日志保留，分母已更正。教师采集首次预检因路径生成了字符串数组而失败，发生在任何仿真之前；V82 改为文本标量路径，沿用相同九个冻结任务，失败日志保留。</p>
<h2>文献与适用边界</h2><p>采用控制增量决策、动力学展开与有限去噪/优化。轨迹是控制的动力学结果。约束处理参考 <a href="https://proceedings.mlr.press/v283/romer25a.html">DPCC</a>；辅助稳态与相对稳态控制代价参考 <a href="https://arxiv.org/html/2406.06157v2">MPC for tracking</a>。这些组合实验不构成整个方法的稳定性或连续时间安全证明。</p>
<p>目前仅种子 5101、开发工况；动态教师模型尚未训练，五种子和 1440 次正式对比尚未完成。正式比较对象仍只有基线与新方法。本页版本比较用于开发诊断，不扩展正式对比方案。</p>
<h2>可复算文件</h2><p><a href="development_results.csv">开发结果 CSV</a> · <a href="summary.json">状态与结论</a> · <a href="source_manifest.json">源码哈希</a> · <a href="artifact_manifest.json">原始文件哈希</a></p>
<p>执行入口：<code>ch5SpeedForceFullV79</code>（当前开发起点）、<code>ch5SpeedTeacherPilotV82</code>（逐任务恢复）、<code>teacher_pilot_v82.py</code>（窗口与数据审计）、<code>report_v82.py</code>（复算报告）。完成的实验目录会拒绝覆盖；重做算法实验使用新版本和目录。</p></html>'''
    (OUT/'index.html').write_text(description,encoding='utf-8')
    print(json.dumps({k:summary[k] for k in ['completed_full_development_runs','raw_candidates_reaudited','preserved_historical_source_count','source_manifest_count','teacher_pilot_completed_states']},ensure_ascii=False))
    print('Report',OUT/'index.html')

if __name__=='__main__':main()
