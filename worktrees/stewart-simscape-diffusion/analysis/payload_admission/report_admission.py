"""Create standalone plots and a readable Chinese audit report."""
import argparse
import csv
import html
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_admission import read

LABELS={'rawModeRms':'原几何模式距离（无量纲）','positionRmsMm':'位置差 RMS / mm',
        'rotationRmsDeg':'姿态差 RMS / °','velocityRmsMmS':'平移速度差 RMS / mm/s',
        'accelerationRmsMmS2':'平移加速度差 RMS / mm/s²','forceRmsN':'六腿需求力差 RMS / N'}

def fmt(v):return f'{v:.4g}'

def main():
    ap=argparse.ArgumentParser();ap.add_argument('folder',type=Path);args=ap.parse_args();p=args.folder
    summary=read(p/'difference_summary.json');choice=read(p/'selection.json')
    with (p/'pairs.csv').open(encoding='utf-8-sig',newline='') as f:pairs=[r for r in csv.DictReader(f) if r['split']=='train']
    plt.rcParams.update({'font.sans-serif':['Microsoft YaHei','DejaVu Sans'],'axes.unicode_minus':False,'font.size':11})
    fig,axes=plt.subplots(2,3,figsize=(14,8),layout='constrained')
    for ax,(metric,label) in zip(axes.flat,LABELS.items()):
        a=np.sort([float(r[metric]) for r in pairs]);a=np.maximum(a,1e-16)
        ax.plot(a,np.arange(1,len(a)+1)/len(a),color='#147d92',lw=2)
        ax.set_xscale('log');ax.set_ylim(0,1.02);ax.set_xlabel(label);ax.set_ylabel('训练集轨迹对累计比例')
        ax.grid(True,alpha=.2);ax.axhline(.95,color='#d39742',ls='--',lw=1)
        ax.set_title('中位数 '+fmt(np.median(a))+'；最大 '+fmt(max(a)),fontsize=10)
    fig.suptitle('同一工况内：拟合合格轨迹的实际差异',fontsize=18)
    fig.savefig(p/'difference_distribution.png',dpi=160);fig.savefig(p/'difference_distribution.svg');plt.close(fig)
    sweep=[r for r in summary['sweep'] if r['metric']=='rawModeRms' and r['cap']==4 and r['threshold']>0]
    fig,(ax,bars)=plt.subplots(1,2,figsize=(13,5),width_ratios=[2,1],layout='constrained')
    ax.semilogx([r['threshold'] for r in sweep],[r['train'] for r in sweep],'o-',color='#147d92',label='仅降低几何去重阈值，每工况最多4条')
    ax.axhline(choice['counts']['train'],color='#a84255',ls=':',label='本次采用：'+str(choice['counts']['train'])+'条')
    ax.invert_xaxis();ax.set_xlabel('几何去重阈值（向右放宽）');ax.set_ylabel('训练标签数（局部放大）');ax.set_ylim(170,230)
    ax.grid(True,alpha=.2);ax.legend(loc='best',fontsize=9);ax.set_title('放宽到0.005后，标签数进入181条的平台区')
    values=[175,choice['counts']['train'],1310]
    bars.bar(['原标签','本次标签','全部合格\n候选'],values,color=['#7b8b96','#147d92','#d39742'])
    for i,v in enumerate(values):bars.text(i,v+20,str(v),ha='center')
    bars.set_ylim(0,1450);bars.set_ylabel('训练轨迹条数');bars.set_title('独立训练工况均为175个')
    fig.savefig(p/'threshold_sweep.png',dpi=160);fig.savefig(p/'threshold_sweep.svg');plt.close(fig)
    stats=summary['statistics']['train']
    separation=read(p/'difference_separation.json');added=read(p/'added_label_differences.json')
    added_rows=''.join('<tr><td>'+html.escape(r['taskId'].replace('formal_train_',''))+'</td>'+''.join('<td>'+fmt(r[m])+'</td>' for m in ['positionRmsMm','rotationRmsDeg','velocityRmsMmS','accelerationRmsMmS2','forceRmsN'])+'</tr>' for r in added)
    rows=''.join('<tr><td>'+html.escape(label)+'</td>'+''.join('<td>'+fmt(stats['pairWeighted'][metric][q])+'</td>' for q in ['median','p95','max'])+'</tr>' for metric,label in LABELS.items())
    gate_rows=''.join(f"<tr><td>{r['threshold']:.6g}</td><td>{r['train']}</td><td>{r['validation']}</td></tr>" for r in sweep)
    report=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>轨迹差异与准入门槛调整</title>
<style>body{{font:16px/1.8 'Microsoft YaHei',sans-serif;color:#253747;max-width:1100px;margin:40px auto;padding:0 24px}}h1,h2{{color:#164e63}}table{{border-collapse:collapse;width:100%;margin:18px 0}}th,td{{text-align:left;border-bottom:1px solid #dce5eb;padding:9px}}th{{background:#eff6f8}}img{{width:100%}}.lead{{background:#eef7f8;padding:20px;border-left:5px solid #147d92}}pre{{white-space:pre-wrap;word-break:break-all;background:#f5f7f9;padding:16px}}small{{color:#64748b}}</style>
<h1>轨迹差异与准入门槛调整</h1>
<div class="lead">原训练集175条 → 本次{choice['counts']['train']}条；新增{len(choice['addedTrainingLabels'])}条。独立训练工况仍为175个，验证标签仍为53条，验证评估工况仍为120个。</div>
<p>重新拟合并按原物理门槛复查全部1,506条候选（训练1,310、验证196）。位置、姿态、速度、加速度和需求力差异在网络实际学习的样条轨迹上、同一时刻比较；每条采用513个等间隔采样点。物理验收另用原5 ms稠密检查及原近障碍加密规则。</p>
<h2>实际差异</h2><p>训练集共有{stats['pairs']}对同工况轨迹，其中{separation['smallDifferencePairs']}对（{100*separation['smallDifferenceFraction']:.2f}%）仅有微小数值差异：这部分的位置RMS差最大也只有{fmt(separation['smallDifferenceMax']['positionRmsMm'])} mm。明显差异集中在5个双工装箱体载荷工况。</p><p>下表按轨迹对统计；每工况最大差异的等工况统计另见difference_summary.json。几何模式距离沿路径进度对齐，其余指标沿相同物理时间对齐。</p>
<table><tr><th>指标</th><th>中位数</th><th>P95</th><th>最大值</th></tr>{rows}</table>
<img src="difference_distribution.png" alt="差异累计分布">
<h2>门槛敏感性</h2><p>表中只改变原几何去重阈值，保持原读取顺序和每工况最多4条。验证列只是敏感性统计，本次交付仍保留原53条验证标签。</p>
<table><tr><th>几何距离阈值</th><th>训练标签</th><th>验证标签（敏感性统计）</th></tr>{gate_rows}</table>
<img src="threshold_sweep.png" alt="标签数随阈值变化">
<h2>本次实际采用的准入策略</h2><p>几何去重阈值由0.15降到0.005（放宽30倍）；保留原读取顺序，每工况仍最多4条。0.005到0.000001均得到181条训练标签，继续降低阈值主要开始纳入数值微差。该阈值是本批训练数据的准入选择，不是硬件分辨率标准。</p>
<p>新增6条与各工况原代表轨迹的差异如下，全部指标为RMS。</p>
<table><tr><th>工况</th><th>位置/mm</th><th>姿态/°</th><th>速度/mm/s</th><th>加速度/mm/s²</th><th>需求力/N</th></tr>{added_rows}</table>
<details><summary>精确策略与复现参数</summary><pre>{html.escape(__import__('json').dumps(choice['policy'],ensure_ascii=False,indent=2))}</pre></details>
<p>原始可行性、拟合复查、间隙、腿运动学、力和奇异性要求保持原值。新增样本不会改变独立工况数；现有训练程序按工况等权采样，再在工况内随机选标签，避免标签较多的工况获得额外权重。</p>
<p>本轮交付为数据集准入调整，未重新训练模型，未据此宣称泛化改善。原模型、数据集和测试结果保留。</p>
<h2>可复核产物</h2><p><a href="dataset_delivery.zip">新版数据集完整包</a> · <a href="../payload_formal_dataset_admission_v2_20260926/dataset.json">训练数据JSON</a> · <a href="../payload_formal_dataset_admission_v2_20260926/verification.json">交付校验</a></p><p><a href="pairs.csv">逐轨迹对差异</a> · <a href="threshold_sweep.csv">阈值扫描</a> · <a href="difference_summary.json">完整统计</a> · <a href="selection.json">实际选择清单</a> · <a href="audit_summary.json">物理复查结果</a></p>
<small>数值来源：冻结正式数据集及其原始离线参考解。完整身份哈希见audit_identity.json和selection.json。</small></html>'''
    (p/'report.html').write_text(report,encoding='utf-8')
    print(p/'report.html')

if __name__=='__main__':main()
