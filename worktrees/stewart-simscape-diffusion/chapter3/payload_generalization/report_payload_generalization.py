"""Reports measured records only. Kept outside the numerical code hash."""
import csv
import html
from pathlib import Path
import numpy as np
from pg_core import read,write

def summarize(folder):
    folder=Path(folder); attempts=[read(f) for f in sorted(folder.glob('attempt_*.json'))]
    if not attempts: raise ValueError('No actual attempt logs in this run')
    fields=['attempt','taskId','baseSceneId','split','shape','mass','T','startId','endpointPassed','passed','reason','actualAttemptSeconds','maxEquality','maxInequality','maxBounds','physicalReason','buildSeconds','solveSeconds','validationSeconds']
    with (folder/'attempts.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore'); writer.writeheader(); writer.writerows(attempts)
    qualified=sum(bool(r['passed']) for r in attempts); tasks=len({r['taskId'] for r in attempts}); feasible_tasks=len({r['taskId'] for r in attempts if r['passed']})
    invalid=sum(not r['endpointPassed'] for r in attempts); exceptions=sum('exceptionStack' in r for r in attempts)
    report=dict(attempts=len(attempts),conditions=tasks,strictQualifiedAttempts=qualified,strictQualifiedConditions=feasible_tasks,endpointRejectedAttempts=invalid,exceptionAttempts=exceptions,
                measuredAttemptSeconds=sum(r['actualAttemptSeconds'] for r in attempts),trainingRuns=0,formalBenchmarkCalls=0)
    write(folder/'observed_summary.json',report)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans','figure.dpi':150})
    fig,axes=plt.subplots(1,2,figsize=(10,3.6))
    classes=['Qualified','Endpoint rejected','Other failure']; counts=[qualified,invalid,len(attempts)-qualified-invalid]
    axes[0].bar(classes,counts,color=['#25746a','#9aa6b3','#cf7854']); axes[0].set_ylabel('Actual attempts (not independent conditions)')
    axes[1].plot([r['attempt'] for r in attempts],[r['actualAttemptSeconds'] for r in attempts],'o-',color='#315d87'); axes[1].axhline(120,color='#bc493d',ls='--',lw=1,label='120 s attempt limit'); axes[1].set(xlabel='Attempt number',ylabel='Actual elapsed seconds'); axes[1].legend()
    fig.suptitle('Payload development: numerical references only'); fig.tight_layout(); fig.savefig(folder/'reference_attempts.png'); fig.savefig(folder/'reference_attempts.svg'); plt.close(fig)
    failures={}
    for r in attempts:
        if not r['passed']: failures[r['reason']]=failures.get(r['reason'],0)+1
    lines=''.join(f'<tr><td>{html.escape(str(k))}</td><td>{v}</td></tr>' for k,v in failures.items())
    doc=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>匀质载荷开发记录</title>
<style>body{{max-width:1050px;margin:36px auto;padding:0 24px;font:16px/1.7 system-ui;color:#243445}}h1{{font-size:27px}}table{{border-collapse:collapse;width:100%}}td,th{{border-bottom:1px solid #d8e0e6;padding:9px;text-align:left}}.note{{background:#eff5f7;padding:18px}}img{{max-width:100%}}</style>
<h1>不同匀质载荷：开发参考求解记录</h1><p>来源：{html.escape(folder.name)}；数据仅来自本目录真实调用日志。</p>
<p class="note">已记录 {len(attempts)} 次尝试，覆盖 {tasks} 个条件；{qualified} 次严格合格，涉及 {feasible_tasks} 个条件。重复求解不增加独立条件数。本页不是检索与扩散比较，也没有泛化性能结论。</p>
<img src="reference_attempts.png" alt="求解结果与真实耗时"><p>严格合格同时要求求解成功状态、等式／不等式／变量界残差 ≤ 10⁻⁶、独立物理检查通过，以及在 120 秒内返回。端点拒绝和超时均保留。</p>
<h2>失败记录</h2><table><tr><th>原因</th><th>尝试数</th></tr>{lines}</table>
<h2>当前可支持的结论</h2><p>新载荷统一模型已取得上述条件的数值可行证据；其覆盖范围受开发任务限制。尚未通过完整新载荷训练与正式对比，不能据此声称扩散优于历史复用。</p>
<p>机构包络是声明的理想化模型，稠密数值验收不是连续时间安全证明。原始 MAT 轨迹、JSON 日志与 <a href="attempts.csv">逐尝试表</a>保存在同一目录。</p></html>'''
    (folder/'implementation_report.html').write_text(doc,encoding='utf-8'); return folder/'implementation_report.html'

if __name__=='__main__':
    import sys
    print(summarize(sys.argv[1]))
