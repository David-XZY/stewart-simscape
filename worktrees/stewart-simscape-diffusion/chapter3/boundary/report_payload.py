"""Render observed payload sensitivity, without inferring diffusion advantage."""
from pathlib import Path
import json
import html
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'results/chapter3/boundary_payload_pilot_v1'


def main():
    pilot = json.loads((OUT / 'summary.json').read_text(encoding='utf-8'))
    reuse = json.loads((OUT.parent / 'boundary_payload_reuse_v1/summary.json').read_text(encoding='utf-8'))
    rows = pilot['rows']
    assert len(rows) == 48 and all(r['endpointPassed'] for r in rows)
    assert sum(r['passed'] for r in rows) == pilot['passed']
    def witnessed(r):
        w = r['gapViolationWitness']
        return isinstance(w, dict) and w['valid'] and w['distanceUpper'] < .01
    n_geom = sum(witnessed(r) for r in rows)
    n_force = sum(r['quality']['forcePeakN'] > 2000 for r in rows)
    n_both = sum(witnessed(r) and r['quality']['forcePeakN'] > 2000 for r in rows)
    assert n_geom + n_force - n_both == len(rows) - pilot['passed']
    fig, ax = plt.subplots(1, 2, figsize=(10.5, 4.2))
    for duration in [7.5, 2]:
        chosen = sorted((r for r in rows if r['parameters']['length'] == 1.2 and r['parameters']['radius'] == .15 and r['parameters']['T'] == duration), key=lambda r:r['parameters']['mass'])
        ax[0].plot([r['parameters']['mass'] for r in chosen], [r['quality']['forcePeakN'] for r in chosen], 'o-', label=f'T = {duration:g} s')
    chosen = [r for r in rows if r['parameters']['length'] == 1.2 and r['parameters']['radius'] == .15 and r['parameters']['T'] == 7.5]
    ax[0].plot([r['parameters']['mass'] for r in chosen], [r['staticForcePeakN'] for r in chosen], ':', label='Endpoint static force')
    ax[0].axhline(2000, color='black', linestyle='--', linewidth=1, label='Actuator limit')
    ax[0].set(xlabel='Payload mass (kg)', ylabel='Peak required leg force (N)', title='Same path; L = 1.2 m, r = 0.15 m')
    ax[0].legend(fontsize=8)
    labels = ['Accepted', 'Gap witness only', 'Force only', 'Both']
    counts = [pilot['passed'], n_geom-n_both, n_force-n_both, n_both]
    bars = ax[1].bar(labels, counts, color=['#2a7f62', '#c68945', '#b84a48', '#6e4f79'])
    for bar, n in zip(bars, counts): ax[1].text(bar.get_x()+bar.get_width()/2, n+.4, str(n), ha='center')
    ax[1].set(ylabel='Diagnostic combinations', title='48 conditions; all endpoints legal', ylim=(0, 32))
    ax[1].tick_params(axis='x', labelsize=8)
    fig.tight_layout()
    for extension in ['png', 'svg']: fig.savefig(OUT / f'payload_sensitivity.{extension}', dpi=180)
    plt.close(fig)
    table = ''.join(f"<tr><td>{r['parameters']['mass']:g}</td><td>{r['parameters']['length']:g}</td><td>{r['parameters']['radius']:g}</td><td>{r['parameters']['T']:g}</td><td>{r['quality']['forcePeakN']:.1f}</td><td>{'通过' if r['passed'] else html.escape(r['reason'])}</td><td>{'有' if witnessed(r) else '无'}</td></tr>" for r in rows)
    doc = f'''<!doctype html><meta charset="utf-8"><title>载荷变化：模型与小试</title>
<style>body{{font:16px/1.75 system-ui,"Microsoft YaHei";max-width:1000px;margin:32px auto;padding:0 20px;color:#233744}}h1{{font-size:26px}}h2{{font-size:20px}}img{{max-width:100%}}table{{border-collapse:collapse;width:100%;font-size:14px}}td,th{{padding:6px 10px;border-bottom:1px solid #dce3e8;text-align:left}}.note{{background:#eef4f8;padding:14px}}</style>
<h1>将载荷质量与尺寸纳入条件：开发小试</h1>
<p class="note">本次实际执行48个固定路径验收、36个旧几何候选验收及1次重新计时。没有新IHSID标签求解、训练或扩散调用。结果说明载荷变化能改变可行性，尚不能说明扩散更好，也未证明失败任务存在另一条合格路径。</p>
<h2>模型与输入</h2>
<p>第一阶段使用均匀圆柱族，改变质量m、长度L和半径r，暂未增加方箱等形状类别。平台、执行器和工装不变。载荷中心安装坐标固定为[0,0,0.15] m，不扫描横向偏心；质量变化后的合成质心和惯量仍按刚体合成正确重算。</p>
<p>圆柱自身惯量为diag(mr²/2, m(3r²+L²)/12, m(3r²+L²)/12)，结合平台惯量及平行轴项得到合成惯量。标称100 kg、1.2 m、0.15 m能精确还原旧模型。质量变化不改变同一位姿的腿长、雅可比和碰撞几何，但会改变逆动力学力。</p>
<p>已新增开发用96维条件：原93维加[m,L,r]。尚未迁移正式训练、检索和任务重建全链条；旧93维权重不能称为适应载荷的新模型。首轮无需增加网络隐藏层。机构包络与安装为理想化声明，不代表真实400 kg额定载荷或接触/自碰撞认证。</p>
<h2>实际结果</h2>
<p>冻结组合为m∈{{50,100,200,400}} kg，L∈{{0.8,1.2,1.3}} m，r∈{{0.10,0.15}} m，T∈{{7.5,2}} s。48个端点均合法，同一路径通过{pilot['passed']}个。{n_geom}个存在间隙违反点对见证，{n_force}个存在力超限，二者重合{n_both}个。</p>
<img src="payload_sensitivity.png">
<p>长度增加至1.3 m时，原路径存在碰撞见证；这条固定几何路径不能靠重新计时消除碰撞。400 kg、1.2 m、r=0.15 m时，端点静态峰值力约1610 N，7.5 s轨迹峰值约1842 N，2 s约2109 N，超过2000 N。这是动态约束失效，并非端点本身已不合法。</p>
<p>另取三个案例：(100 kg,1.3 m,7.5 s)、(400 kg,1.2 m,2 s)、(400 kg,1.3 m,2 s)，半径均0.15 m。此前12条合格几何探针在每个案例均为0/12通过。该集合不是训练历史库，不代表全库失败。质量案例的单次12控制点/40迭代重新计时以{html.escape(reuse['retiming']['solve']['status'])}结束，耗时{reuse['retiming']['solve']['seconds']:.2f} s，物理复查未通过；这不能证明其他重新计时不可能。</p>
<h2>建议的新实验主线</h2>
<p>将问题改为“固定节拍下，载荷尺寸与惯性变化时的条件轨迹生成”。先做质量单因素、尺寸单因素、质量—尺寸—节拍联合变化三组；寻找端点合法、至少一个独立规划器可解、旧路径较难复用的区域。首先补独立可行解证据及严格合格IHSID标签，随后才启动新训练。</p>
<p>训练与检索使用相同新数据，学习方法和增强检索均接收m、L、r。质量与几何尺度仅用训练集标定；重新计时必须重算逆动力学。测试同时包含域内新条件与未见参数组合，外推另列。固定参考优化目标、验收门及在线预算保持一致。新增一种参数本身不构成扩散独有优势；仍须比较增强检索、普通回归、重新计时及共同精修。</p>
<p>“载荷条件扩散”已有相关研究：<a href="https://arxiv.org/abs/2508.21375">Dynamics-Compliant Trajectory Diffusion for Super-Nominal Payload Manipulation（CoRL 2025）</a>，对象为串联机械臂；不能直接推导本Stewart平台的收益。</p>
<h2>逐条件记录</h2><table><thead><tr><th>质量/kg</th><th>长度/m</th><th>半径/m</th><th>T/s</th><th>峰值力/N</th><th>原路径验收</th><th>间隙违反见证</th></tr></thead><tbody>{table}</tbody></table>
<p>源码：bdPayloadTask.m、bdPayloadPilot.m、bdPayloadReuseProbe.m。数据：本目录manifest.json、summary.json、task_*.json，以及相邻boundary_payload_reuse_v1目录。报告重建：python chapter3/boundary/report_payload.py。低分离下界本身不能证明碰撞；表中见证来自实体内点对的距离上界。</p>'''
    (OUT / 'payload_report.html').write_text(doc, encoding='utf-8')
    print(OUT / 'payload_report.html')


if __name__ == '__main__':
    main()
