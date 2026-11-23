"""Render only observed diagnosis/continuation records; excluded from numerical hash."""
import argparse
import html
import json
from pathlib import Path
import datetime

def read(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def report(results,output):
    results=Path(results);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    diagnosis=read(results/'payload_diagnosis_v2/diagnosis_report.json')
    job=results/'payload_continuation_v2';status=read(job/'status.json') if (job/'status.json').exists() else {}
    direct=[read(f) for f in sorted((job/'direct').glob('*.json'))]
    trained=[]
    for f in sorted((results/'payload_models_v2').glob('*/model.json')):
        a=read(f);trained.append(dict(name=f.parent.name,method=a['method'],seed=a['seed'],bestPhysicalRate=a['bestPhysicalRate'],bestEpoch=a['bestEpoch'],updates=a['optimizerUpdates'],seconds=a['elapsedSeconds'],validationSeconds=a['validationSeconds']))
    benchmarks=[read(f) for f in sorted(job.glob('*_summary.json'))]
    from pg_statistics import paired_bootstrap
    for b in benchmarks:
        calls=[read(f) for f in (results/b['run']/'calls').glob('*/result.json')]
        if not calls:continue
        b['pairedSceneBootstrap']=[]
        methods=sorted({r['method'] for r in calls})
        if 'retrieval' in methods:
            for budget in sorted({r['B'] for r in calls}):
                for method in methods:
                    if method=='retrieval':continue
                    contrast=paired_bootstrap(calls,method,'retrieval',group='development',budget=budget)
                    b['pairedSceneBootstrap'].append(dict(method=method,baseline='retrieval',B=budget,**contrast))
        b['auditCoverage']=dict(calls=len(calls),onlinePassed=sum(r['onlinePassed'] for r in calls),finalQualified=sum(r['finalQualified'] for r in calls),
                               denseAudited=sum(r['onlinePassed'] for r in calls),fineAudited=sum(bool(r.get('fineAuditSelected')) for r in calls),
                               fineFailed=sum(r.get('fineAuditSelected',False) and not r.get('fineAuditPassed',False) for r in calls))
    data=dict(diagnosis=diagnosis,training=trained,direct=direct,benchmarks=benchmarks,jobStatus=status,scope='Development only. No formal test conclusion.')
    (output/'observed_results.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    series=[('Original',diagnosis['summary']['original'])]
    for d in direct:
        if d['model']=='ddpm_basic_20260919_update_only':series.append(('More updates\n(no projection)',d))
    series.append(('Projection only\n(same weights)',diagnosis['summary']['projection_only_same_weights']))
    for d in direct:
        if d['model']=='ddpm_basic_20260919':series.append(('More updates\n+ projection',d))
    fig,ax=plt.subplots(figsize=(9,3.8),layout='constrained')
    vals=[100*r['passed']/r['count'] for _,r in series]; bars=ax.bar([n for n,_ in series],vals,color=['#ae5b54','#8494a7','#bc9244','#397d78'][:len(series)])
    for bar,(_,r) in zip(bars,series):ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+2,f"{r['passed']}/{r['count']}",ha='center')
    ax.set_ylim(0,100);ax.set_ylabel('Direct physical pass rate (%)');ax.set_title('Paired diagnostic replay: training seed 20260919; no repair or fallback')
    fig.savefig(output/'diagnosis.png',dpi=170);fig.savefig(output/'diagnosis.svg');plt.close(fig)
    esc=lambda x:html.escape(str(x))
    rows=''.join(f"<tr><td>{esc(r['name'])}</td><td>{r['updates']}</td><td>{r['bestEpoch']}</td><td>{r['bestPhysicalRate']:.1%}</td><td>{r['seconds']:.1f}</td><td>{r['validationSeconds']:.1f}</td></tr>" for r in trained)
    dr=''.join(f"<tr><td>{esc(r['model'])}</td><td>{r['passed']}/{r['count']}</td><td>{r['stratumEqualRate']:.1%}</td></tr>" for r in direct)
    br=''.join(f"<h3>{esc(b['run'])}</h3><pre>{esc(json.dumps(b['summary'],indent=2))}</pre>" for b in benchmarks)
    timestamp=datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    doc=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>载荷扩散：问题定位与后续验证</title>
<style>body{{font:16px/1.75 system-ui;color:#203344;max-width:1080px;margin:32px auto;padding:0 24px}}h1{{font-size:28px}}h2{{font-size:21px;margin-top:30px}}.note{{background:#edf3f5;border-left:4px solid #397d78;padding:16px}}table{{width:100%;border-collapse:collapse;font-size:14px}}td,th{{border-bottom:1px solid #d5dfe4;padding:8px;text-align:left}}img{{max-width:100%}}pre{{white-space:pre-wrap;background:#f4f6f7;padding:12px}}</style>
<h1>匀质载荷扩散：问题定位与后续验证</h1><p>报告生成：{timestamp}。以下是回收时刻的实测快照，不是实时监控。</p>
<p class="note">研究处于开发／验证阶段。旧模型与结果保留，修正结果使用 v2 独立目录。正式泛化测试尚未开展，当前不能宣称扩散优于增强检索。</p>
<h2>已定位的问题</h2><ol><li>65 个训练条件不足一个 batch，旧代码每轮只有一次更新；旧扩散仅训练 150–200 次更新。</li>
<li>cosine 日程最后一步由噪声预测恢复干净样本时，误差放大系数约 {diagnosis['terminalCleanAmplification']:.0f}。没有采样范围约束时，控制量达到训练尺度约千倍，导致腿长等约束失败。</li>
<li>统一验收入口的可选参数计数错误会清空 IHSID 求解证据。已修正，并增加真实求解记录的接受、假成功状态、变量界残差与超时拒绝测试；离线参考标签不经过该错误入口。</li>
<li>公共精修／引导原先每任务重新建图。现仅缓存任务无关的参数化图，端点、时长、质量、惯量、尺寸及障碍全部按当前任务传入。</li></ol>
<h2>原因验证</h2><img src="diagnosis.png" alt="同开发验证集直接通过率对照">
<p>同一模型仅加入训练集定义的采样范围约束：0/64 → 12/64。它是明确披露的采样稳定化，并非物理可行证书。图中使用预先固定的诊断种子 20260919；下表保留全部已完成种子，不能把单个种子的改善当作稳定领先。全部 100 条合格标签重建复查通过；共享噪声下 Python/MATLAB 参数最大误差 {diagnosis['productionSamplerMaxAbsoluteError']:.3g}，候选前缀一致。</p>
<h2>重训记录</h2><p>新配置至少 32 次更新／轮，早停前至少 2000 次更新；所有学习对照共同使用该训练工作量规则。检查点依据固定 24 条开发验证任务选择，不能与下面完整 64 条任务结果混用。</p>
<table><tr><th>模型</th><th>总更新</th><th>最佳轮次</th><th>24 条检查点通过率</th><th>总秒数</th><th>验证秒数</th></tr>{rows}</table>
<h2>完整开发验证集直接生成</h2><table><tr><th>模型</th><th>通过／全部条件</th><th>场景类别与形状等权率</th></tr>{dr}</table>
<h2>真实在线预算对照</h2>{br or '<p>尚无已完成的整组结果；不把内部采样或检查点验证当作在线返回率。</p>'}
<h2>运行与恢复</h2><p>远程状态：{esc(status.get('status','unknown'))}；阶段：{esc(status.get('stage','unknown'))}。新控制进程由 Windows WMI 启动，其父进程是 WmiPrvSE，已在启动 SSH 会话退出后确认仍运行。每十轮原子保存模型、优化器、随机状态、最佳检查点和累计耗时；模拟中断后恢复结果与连续训练逐参数一致。在线对照按完成调用续跑，已完成结果不覆盖；进程或机器崩溃后的重新启动仍需执行续跑入口，不声称自动恢复供电或重启。</p>
<h2>继续范围</h2><p>自动队列重跑检索选择，完成基础三种子对照，然后进行物理签名、容量与特征消融、引导与末端修正、物理损失权重选择，以及条件触发的组合版本。全部选择只使用开发验证任务。正式数据规模和正式协议在开发验证完成后审查，不会偷偷读取正式测试答案。</p>
<p>物理验收仍要求原有全部约束；数值稠密检查不构成连续时间安全证明。机器可读的全部观察记录见 <a href="observed_results.json">observed_results.json</a>。</p></html>'''
    path=output/'implementation_report.html';path.write_text(doc,encoding='utf-8');return path

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--results',required=True);p.add_argument('--output',required=True);a=p.parse_args();print(report(a.results,a.output))
