"""Summarize pilot evidence without treating sampling as certification or formal testing."""
from pathlib import Path
import argparse
import csv
import hashlib
import html
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import scipy.io as sio
from control_force import OUT, CH5, write_json


def describe(values):
    values=np.asarray(values,dtype=float)
    values=values[np.isfinite(values)]
    if not len(values): return None
    return dict(median=float(np.median(values)),p95=float(np.percentile(values,95)),maximum=float(np.max(values)))


def main(variant='v60'):
    directory=OUT/'pilot'
    report=dict(revision='force-'+variant+'-development',model_revision='force-v57-development',formal_ready=False,formal_runs=0,terminal_certified=False)
    report['algebra']=json.loads((OUT/'algebra.json').read_text('utf-8'))
    for name,path in [('teacher_coverage',OUT/'teacher_coverage.json'),('admitted_coverage',directory/'coverage.json'),('resume_check',OUT/'resume_check/report.json'),('moving_teacher_failures',OUT/'teacher_moving_failure_audit.json')]:
        if path.exists(): report[name]=json.loads(path.read_text('utf-8'))
    labels=json.loads((directory/'label_report.json').read_text('utf-8'))
    report['labels']={s:dict(requested=sum(r['split']==s for r in labels['rows']),
        raw_accepted=sum(r['split']==s and r['raw_check']['accepted'] for r in labels['rows']),
        repaired_accepted=sum(r['split']==s and r['repaired_check']['accepted'] for r in labels['rows'])) for s in ['train','validation']}
    report['label_force_change_N']=describe([r['force_rms_change_N'] for r in labels['rows'] if r['repaired_check']['accepted']])
    report['regions']=[dict(mode=r['mode'],rho=r['terminal']['rho'],pole_radius=r['terminal']['linear_pole_radius'],
        sampled_check=r['terminal']['region_verification']) for r in labels['regions']]
    candidate_directory=directory/('candidate_'+variant)
    if (candidate_directory/'candidate_report.json').exists():
        candidates=json.loads((candidate_directory/'candidate_report.json').read_text('utf-8'));report['parity']=candidates['parity']
        rows=candidates['rows'];report['candidates']=dict(total=len(rows),raw_accepted=sum(r['raw_check']['accepted'] for r in rows),
            raw_physical_accepted=sum(r['raw_check']['finite'] and r['raw_check']['inequality_violation'] is not None and r['raw_check']['inequality_violation']<=1e-6 for r in rows),
            projected_accepted=sum(r['projected_check']['accepted'] for r in rows),solver_failures=sum(not r['solve']['solver_success'] for r in rows),
            force_error_N=describe([r['force_rms_error_N'] for r in rows]),increment_error_N=describe([r['increment_rms_error_N'] for r in rows]),
            batch_projection_seconds=describe([r['batch_projection_seconds'] for r in rows][::4]))
        with (candidate_directory/'candidate_summary.csv').open('w',newline='',encoding='utf-8-sig') as f:
            fields=['window_index','mode','seed','candidate','force_rms_error_N','increment_rms_error_N','line']
            writer=csv.DictWriter(f,fieldnames=fields+['raw_accepted','projected_accepted','solver_success']);writer.writeheader()
            for r in rows: writer.writerow({**{k:r[k] for k in fields},'raw_accepted':r['raw_check']['accepted'],
                'projected_accepted':r['projected_check']['accepted'],'solver_success':r['solve']['solver_success']})
        fig,axs=plt.subplots(1,2,figsize=(9,3.8))
        axs[0].hist([r['force_rms_error_N'] for r in rows],bins=20);axs[0].set_xlabel('Raw force RMS error (N)');axs[0].set_ylabel('Candidates')
        axs[1].bar(['Raw full gate','Projected full gate'],[report['candidates']['raw_accepted'],report['candidates']['projected_accepted']]);axs[1].set_ylim(0,len(rows));axs[1].set_ylabel('Accepted / '+str(len(rows)))
        fig.tight_layout();fig.savefig(directory/('candidates_'+variant+'.png'),dpi=150);plt.close(fig)
    metadata=directory/'seed_5101/metadata.json'
    if metadata.exists(): report['training']=json.loads(metadata.read_text('utf-8'))
    closed=directory/('closed_loop_'+variant)/'summary.json'
    if closed.exists():
        r=json.loads(closed.read_text('utf-8'));report['closed_loop_completed']=r['completed'];report['closed_loop_error']=r['error'];report['closed_loop']=[]
        for row in r['rows']:
            report['closed_loop'].append(dict(task=row['task'],mode=row['mode'],safe_success=row['metrics']['safe_success'],
                final_position_mm=row['metrics']['final_position']*1000,wall_seconds=row['wall_seconds'],
                fallback=row['metrics']['fallback_count'],solver_failures=row['metrics']['predictive_solver_failures'],
                refresh_seconds=describe(row['refresh_seconds']),execution_seconds=describe(row['execution_seconds']),
                refresh_misses=row['refresh_deadline_misses'],execution_misses=row['execution_deadline_misses'],
                raw_accepted=row['raw_accepted'],projected_accepted=row['projected_accepted']))
        fig,axes=plt.subplots(2,2,figsize=(11,7.2));colors=['#1b6aa5','#cf5a28']
        for i,row in enumerate(r['rows']):
            file=closed.parent/f"{row['task']:03d}_{row['mode'].replace('-','_')}.mat"
            data=sio.loadmat(file,simplify_cells=True,variable_names=['audit','records','predictions'])
            a=data['audit'];label=f"task {row['task']}"
            axes[0,0].plot(a['time'],1000*a['position_error'],color=colors[i],label=label)
            axes[0,1].plot(a['time'],np.rad2deg(a['rotation_error']),color=colors[i],label=label)
            axes[1,0].plot(a['time'],np.sqrt(np.mean(a['actual_force']**2,axis=1)),color=colors[i],label=label)
            times=np.array([v['time'] for v in data['records']]);seconds=np.array([v['controller_seconds'] for v in data['records']])
            refresh=np.array([v['prediction']['refresh'] for v in data['predictions']],dtype=bool)
            axes[1,1].plot(times[refresh],1000*seconds[refresh],'.',color=colors[i],label=label+' refresh')
            axes[1,1].plot(times[~refresh],1000*seconds[~refresh],'.',color=colors[i],alpha=.35,label=label+' execute')
        axes[0,0].axhline(5,color='#777',linestyle='--',linewidth=1);axes[0,0].set_ylabel('Position error (mm)')
        axes[0,1].axhline(.3,color='#777',linestyle='--',linewidth=1);axes[0,1].set_ylabel('Geodesic rotation error (deg)')
        axes[1,0].set_ylabel('Actual six-leg force RMS (N)')
        axes[1,1].axhline(100,color='#777',linestyle='--',linewidth=1);axes[1,1].axhline(20,color='#aaa',linestyle=':',linewidth=1)
        axes[1,1].set_yscale('log');axes[1,1].set_ylabel('Controller call (ms)')
        for ax in axes.ravel():ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.2);ax.legend(fontsize=8)
        fig.suptitle('Development Simscape: model V57 / controller '+variant.upper()+' / seed 5101')
        fig.tight_layout();fig.savefig(directory/('closed_loop_'+variant+'.png'),dpi=150);plt.close(fig)
    h=hashlib.sha256()
    for p in sorted(p for p in CH5.iterdir() if p.suffix in ['.m','.py','.json']): h.update(p.name.encode());h.update(p.read_bytes())
    report['production_guard']=h.hexdigest()
    assert h.hexdigest()=='a0735e0e37317afcf4107359797e9fbab71fcb886584beac318b42edbcb050d6'
    initial=directory/'closed_loop_v58/summary.json'
    if initial.exists():
        failed=json.loads(initial.read_text('utf-8'))
        report['retained_v58_failure']=dict(error=failed['error'],rows=[dict(task=r['task'],mode=r['mode'],safe_success=r['metrics']['safe_success'],final_position_mm=1000*r['metrics']['final_position']) for r in failed['rows']])
    interfaces=directory/'interfaces_v59.json'
    if interfaces.exists(): report['interfaces']=json.loads(interfaces.read_text('utf-8'))
    files=[*CH5.joinpath('reduced').glob('*V5[789].m'),*CH5.joinpath('reduced').glob('*V60.m'),*Path(__file__).parent.glob('*.py'),*Path(__file__).parent.glob('*.json')]
    report['source_sha256']={str(f.relative_to(CH5)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
    write_json(directory/('summary_'+variant+'.json'),report)
    text=html.escape(json.dumps(report,ensure_ascii=False,indent=2))
    table='<table><tr><th>任务</th><th>安全到达</th><th>终点误差 mm</th><th>墙钟 s</th><th>规划中位 ms</th><th>规划 P95 ms</th><th>回退</th></tr>'
    for row in report.get('closed_loop',[]):
        table+=f"<tr><td>{row['task']} / {row['mode']}</td><td>{'通过' if row['safe_success'] else '未通过'}</td><td>{row['final_position_mm']:.3f}</td><td>{row['wall_seconds']:.2f}</td><td>{1000*row['refresh_seconds']['median']:.2f}</td><td>{1000*row['refresh_seconds']['p95']:.2f}</td><td>{row['fallback']}</td></tr>"
    table+='</table>'
    (directory/('index_'+variant+'.html')).write_text('''<!doctype html><html lang="zh"><meta charset="utf-8"><title>控制增量扩散开发验证</title>
<style>body{font:16px/1.6 system-ui;max-width:1050px;margin:40px auto}pre{white-space:pre-wrap;background:#f3f5f7;padding:20px}img{width:100%}table{border-collapse:collapse;width:100%}td,th{border:1px solid #ddd;padding:8px;text-align:left}</style>
<h1>控制增量扩散：实施与开发验证</h1>
<p>扩散坐标为相对上一拍的完整力序列；通过差分还原控制增量。使用 v 预测和精确零末步信噪比，避免原先 epsilon 重建除以极小 sqrt(alpha)。</p>
<p>终端状态增广上一拍输入，采用局部 Riccati 终端代价、可行稳态和数值验证的椭球域。采样验证不能证明全域不变性。在线四候选共享物理约束线性化，最终接受由完整非线性展开、内部步长、执行器瞬态和终端反馈步的检查决定。失败恢复使用现有安全 QP，其失败或保持上一输入不构成安全保证。</p>
<p>标签来自固定选取的基线窗口，经终端初始化与必要的约束优化修复。保留原始标签、修改量、拒绝记录和来源。训练使用低噪声局部物理敏感度辅助损失；尚未用精确非线性动力学反向传播微调。单种子开发模型不替代五种子正式协议。</p>
<p>文献：<a href="https://arxiv.org/html/2305.08891v4">零信噪比与 v 预测</a>；<a href="https://arxiv.org/html/2406.06157v2">MPC for tracking</a>；<a href="https://proceedings.mlr.press/v283/romer25a.html">DPCC</a>。这些文献不证明当前组合实现的稳定性。</p>
<p>下列计数为开发窗口和开发闭环。正式运行仍为 0/1440。100ms/20ms 是统计目标，不是已验证的实时保证。</p>
<p>本轮数据审计发现：原教师训练集 31901 个窗口、验证集 8032 个窗口均无已观测运动障碍。运动任务的失败运行经过原整任务成功筛选被排除。V60 静态开发任务到达，移动障碍任务安全但未到达。后续全量训练应先补齐成功且物理有效的移动障碍教师任务；当前模型与结果不可视为完整动态泛化验证。</p>
<p>V57 保留数值调试失败记录；V58 保留安全但未到达的闭环。V59 验证导数复用不改变约束和候选。V60 增加力序列非线性终端修正，每次修正后重新检查全时域。不同版本结果分别保存。</p>
'''+table+'''<img src="closed_loop_'''+variant+'''.png" alt="真实 Simscape 时序与计算耗时"><img src="candidates_'''+variant+'''.png" alt="候选误差和接受数"><details><summary>全部数值、训练身份与源码哈希</summary><pre>'''+text+'</pre></details></html>',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},ensure_ascii=False,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--variant',default='v60');main(parser.parse_args().variant)
