"""Current evidence supplement; distinguish identical-sample reuse from reruns."""
from pathlib import Path
import csv,hashlib,json,html,subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from report_raw_v74 import main as report75

ROOT=Path(__file__).resolve().parents[3];ART=ROOT/'chapter5/artifacts/control_force_v62';OUT=ART/'raw_v76'
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    report75(75)
    old=read(ART/'raw_v75/summary.json');probe=read(OUT/'conditioning_probe.json');audit=read(OUT/'conditioning_audit.json');metadata=read(OUT/'seed_5101/metadata.json')
    cond=[]
    for neutralized in [False,True]:
        rr=[r for r in audit['rows'] if r['neutralized']==neutralized]
        cond.append(dict(neutralized=neutralized,count=len(rr),full_accepted=sum(r['check']['accepted'] for r in rr),
            internal_physics_accepted=sum(r['check']['internal_check']['accepted'] for r in rr),
            mechanical_accepted=sum(r['check']['internal_check']['mechanical_violation']<=1e-6 for r in rr)))
    paired=[]
    for version in [75,76]:
        a=read(ART/f'full_v{version}/summary.json');assert a['completed'],'Require all requested development episodes.'
        rr=[r for r in a['rows'] if r['task']==4 and r['mode']=='nonideal-force'];assert len(rr)==1
        r=rr[0];m=r['metrics'];paired.append(dict(version=version,safe=m['all_constraints_valid'],arrival=m['arrival'],position_mm=m['final_position']*1000,
            fallback=m['fallback_count'],raw_accepted=r['raw_accepted'],raw_audited=r['raw_audited'],wall_seconds=r['wall_seconds'],refresh_median_ms=float(np.median(r['refresh_seconds'])*1000),
            actual_end_seconds=m['simulated_end_seconds'],simulation_error=r['simulation_error']))
    h=hashlib.sha256()
    for p in sorted((ROOT/'chapter5').iterdir(),key=lambda p:p.name.lower()):
        if p.is_file() and p.suffix in ['.m','.py','.json']:h.update(p.name.encode());h.update(p.read_bytes())
    guard=dict(root_direct_source_sha256=h.hexdigest(),expected='a0735e0e37317afcf4107359797e9fbab71fcb886584beac318b42edbcb050d6',
        tracked_changes=subprocess.run(['git','diff','--name-only'],cwd=ROOT,capture_output=True,text=True,check=True).stdout.splitlines(),
        git_head=subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,capture_output=True,text=True,check=True).stdout.strip())
    guard['root_preserved']=guard['root_direct_source_sha256']==guard['expected'];assert guard['root_preserved'] and not guard['tracked_changes']
    manifest={}
    for pattern in ['chapter5/reduced/*V73.m','chapter5/reduced/*V74.m','chapter5/reduced/*V75.m','chapter5/reduced/*V76.m','chapter5/reduced/force_v62/*v7*.py','chapter5/reduced/force_v62/RAW_DIFFUSION.md']:
        for p in ROOT.glob(pattern):manifest[str(p.relative_to(ROOT))]=sha(p)
    for p in (ROOT/'chapter5/reduced').rglob('*'):
        if p.is_file() and p.suffix in ['.m','.py','.json','.md']:manifest[str(p.relative_to(ROOT))]=sha(p)
    for p in [OUT/'seed_5101/model.pt',OUT/'seed_5101/weights.mat',OUT/'conditioning_audit.json',OUT/'conditioning_probe.mat',OUT/'finalized.json',ART/'full_v76/004_nonideal_force.mat']:
        if p.exists():manifest[str(p.relative_to(ROOT))]=sha(p)
    data=dict(completed=True,formal_ready=False,formal_runs=0,formal_required=1440,
        fixed_96_result_source='../raw_v75/raw_report.json',fixed_96_not_rerun=True,
        unchanged_fixed_96_verified=metadata['fixed_96_sample_max_difference']==0,prior_fixed_96_summary=old['candidate_summary'],
        conditioning_probe=probe,conditioning_acceptance=cond,moving_paired=paired,metadata=metadata,workspace_guard=guard,
        finalized=read(OUT/'finalized.json'),source_manifest=manifest)
    (OUT/'summary.json').write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    (OUT/'source_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    (OUT/'workspace_guard.json').write_text(json.dumps(guard,indent=2),encoding='utf-8')
    for name,rows in [('conditioning_comparison.csv',cond),('moving_paired.csv',paired)]:
        with (OUT/name).open('w',newline='',encoding='utf-8-sig') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    fig,ax=plt.subplots(1,2,figsize=(8,3.4));ax[0].bar(['Original','Neutralized'],[x['full_accepted']/x['count'] for x in cond]);ax[0].set_ylim(0,1);ax[0].set_ylabel('Moving-condition raw full pass (n=28)')
    ax[1].bar(['V75','V76'],[x['raw_accepted']/x['raw_audited'] for x in paired]);ax[1].set_ylim(0,1);ax[1].set_ylabel('15 s moving-run raw full pass');fig.tight_layout();fig.savefig(OUT/'conditioning.png',dpi=180);plt.close(fig)
    def table(items):
        keys=list(items[0]);return '<table><tr>'+''.join('<th>'+html.escape(k)+'</th>' for k in keys)+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(str(r[k]))+'</td>' for k in keys)+'</tr>' for r in items)+'</table>'
    body='''<!doctype html><html lang="zh-CN"><meta charset="UTF-8"><title>原始扩散诊断与修复 V73–V76</title><style>body{font:16px/1.65 system-ui;max-width:1120px;margin:35px auto;padding:0 20px;color:#233}table{border-collapse:collapse;font-size:13px}th,td{padding:6px;border:1px solid #ccd}img{max-width:100%}</style><h1>原始扩散候选诊断与修复</h1>
    <p>开发研究，正式实验 0/1440。既有 MATLAB worker PID 38888 持续使用，原工程与旧结果保留。模型仅为种子 5101，移动教师训练覆盖仍为零。</p><h2>已完成的主要修复</h2>
    <p>采用可逆的执行器增广反馈创新坐标，保留直接控制增量与真实动力学展开；补齐训练物理约束类别，纠正辅助终端位姿导数，直接训练实际使用的 20 次 DDIM 采样图。固定 96 个条件及噪声的完整通过数从 0 提高到 87，内部物理通过数从 3 提高到 92，均未在生成后限幅、QP 投影或修复尾部。详见<a href="../raw_v75/index.html">V75 全部原始记录、扰动、训练、失败及闭环报告</a>。</p>
    <h2>未观测条件输入的随机响应</h2><p>训练集中有 66 个恒定条件列，其中包含所有障碍速度列。这些列的归一化输入在训练时为零，相关权重无法从数据辨识；推理时 15 mm/s 会被标准差下限映射为 15，触发随机初始化残留的响应。仅改变障碍速度就造成创新输出变化 3.08 N RMS。</p>
    <p>V76 将这些列的输入权重固定为零，诊断记录哪些当前条件未被训练覆盖。物理优化仍读取完整障碍观测与恒速度预测，不掩盖动态约束。此处理不提供动态障碍学习能力；须采集成功且物理有效的移动教师重新训练。</p>
    <p>固定旧验证条件的 96 个 PyTorch 样本与 V75 逐值相同，未重复宣称一次新的 96 次物理实验。新移动条件通过 MATLAB/Python 一致性验证，并单独运行 28 个同噪声候选进行原始完整检查。</p><img src="conditioning.png"><h2>同观测、同噪声的条件输入实验</h2>'''+table(cond)+'''<h2>相同 15 s 移动障碍任务的配对闭环</h2>'''+table(paired)+'''<p>原始候选通过与到达是不同指标。安全约束检查通过不代表已达到任务目标，任何未到达均保留。两版都审计四个原始候选，因此计时包含开发诊断；未达到 10 Hz 墙钟实时性。计算耗时与 15 s 仿真时间分别报告。</p>
    <h2>证据和复现</h2><p><a href="summary.json">结果与边界</a> · <a href="source_manifest.json">代码和模型哈希</a> · <a href="run_manifest.json">保存实验文件哈希</a> · <a href="verification.json">最终逐文件和运行核验</a> · <a href="conditioning_audit.json">全部条件候选检查</a> · <a href="moving_paired.csv">配对闭环 CSV</a> · <a href="finalized.json">同一 MATLAB 会话恢复与清理</a></p>
    <p><a href="https://proceedings.mlr.press/v283/romer25a.html">DPCC</a>以显式模型约束处理保证组合预测的可行性，不能据此宣称未处理的网络样本安全。<a href="https://tml.stanford.edu/PDP.github.io/">PDP</a>讨论物理模仿的误差累积及扰动恢复教师；本轮的局部预稳定与输入可辨识性修复仍需更广泛、独立的闭环与正式测试。</p></html>'''
    (OUT/'index.html').write_text(body,encoding='utf-8')
    print(json.dumps(dict(cond=cond,paired=paired,root_preserved=guard['root_preserved'])));print('Report',OUT/'index.html')
if __name__=='__main__':main()
