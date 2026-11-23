"""Build an auditable repair report from completed independent checks."""
from __future__ import annotations

import csv
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
ART = ROOT / 'artifacts'
OUT = ART / 'repair_report'


def main():
    acceptance = json.loads((ART / 'validation' / 'repair_acceptance.json').read_text('utf-8'))
    assert acceptance['passed'], 'Do not present unfinished acceptance as completed.'
    identity = json.loads((ART / 'runtime_source_identity.json').read_text('utf-8'))
    probes = []
    for path in (ART / 'validation').glob('repair_*_report.json'):
        value = json.loads(path.read_text('utf-8'))
        if value.get('diffusion_source_sha256') == identity['diffusion_source_sha256']:
            assert value['passed']
            probes.append((path, value))
    assert len(probes) == 4, 'All four independent current-source prefixes are required.'
    assert any(v['goal_updates_observed'] > 0 for _, v in probes), 'Goal update was not actually exercised.'
    assert any(v['obstacle_visibility_changes'] > 0 for _, v in probes), 'Obstacle appearance was not actually exercised.'
    OUT.mkdir(exist_ok=True)
    summary = {'control_revision': acceptance['control_revision'], 'diffusion_source_sha256': identity['diffusion_source_sha256'],
               'passed': True, 'prefixes': [v for _, v in probes], 'resource_phases': acceptance['phases'],
               'formal_comparison_complete': False,
               'boundary': 'Independent development prefixes and numeric/API checks; no formal superiority conclusion.'}
    (OUT / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), 'utf-8')
    fields = ['index', 'mode', 'duration', 'control_calls', 'accepted_calls', 'fallbacks',
              'projection_exceptions', 'all_constraints_valid', 'goal_updates_observed', 'obstacle_visibility_changes', 'wall_seconds']
    with (OUT / 'prefix_metrics.csv').open('w', newline='', encoding='utf-8-sig') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields);writer.writeheader()
        writer.writerows({k: v[k] for k in fields} for _, v in probes)
    phases = acceptance['phases']
    with (OUT / 'memory_metrics.csv').open('w', newline='', encoding='utf-8-sig') as handle:
        writer = csv.DictWriter(handle, fieldnames=['phase', 'tree_peak_gib', 'compiler_peak_gib', 'minimum_commit_headroom_gib'])
        writer.writeheader()
        for p in phases:
            writer.writerow(dict(phase=p['name'], tree_peak_gib=p['peak_tree_private_bytes']/2**30,
                                 compiler_peak_gib=p['peak_compiler_private_bytes']/2**30,
                                 minimum_commit_headroom_gib=p['minimum_available_commit_bytes']/2**30))
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.bar([p['name'] for p in phases], [p['peak_tree_private_bytes']/2**30 for p in phases], label='Process tree')
    ax.bar([p['name'] for p in phases], [p['peak_compiler_private_bytes']/2**30 for p in phases], label='Compiler component')
    ax.set_ylabel('Private memory (GiB)');ax.tick_params(axis='x', labelrotation=20);ax.legend()
    fig.tight_layout();fig.savefig(OUT / 'memory.png', dpi=160);plt.close(fig)
    rows = ''.join('<tr>' + ''.join(f'<td>{html.escape(str(v[k]))}</td>' for k in fields) + '</tr>' for _, v in probes)
    headers = ''.join(f'<th>{k}</th>' for k in fields)
    page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>内存与候选求解修复验收</title>
<style>body{{font:16px system-ui;margin:35px;line-height:1.65}}table{{border-collapse:collapse;font-size:13px}}th,td{{border:1px solid #ccc;padding:6px}}img{{max-width:100%}}code{{overflow-wrap:anywhere}}</style>
<h1>内存与候选求解修复验收</h1>
<p>控制修订：{html.escape(acceptance['control_revision'])}；全部独立修复检查通过。完整正式对比仍由实验流水线完成。</p>
<p>改动：串行且每次运行释放 MATLAB；仅保留当前障碍签名的优化器；复用非线性图；超大装配函数使用 O0，较小动力学和导数内核保留优化；保留跨回退的移位初值并提供制动初值；拒绝非有限展开、奇异平衡映射及不可行证书；使用 OSQP 的无穷边界标记；增加求解预算。</p>
<p>物理约束、终端验收、任务规则与五个模型权重保持原定义。所有已选候选重新通过完整动力学及内部积分点验收。有限次求解没有全局最优或整体稳定性保证。</p>
<p>源码：<code>{identity['diffusion_source_sha256']}</code></p>
<h2>独立 Simscape 前缀</h2><p>目标变化采用明确记录的提前事件开发探针。前缀不计入 144 次开发或 1440 次正式运行，不能据此推断完整到达率或新方法优于基线。</p>
<table><thead><tr>{headers}</tr></thead><tbody>{rows}</tbody></table>
<h2>实测内存</h2><p>每个阶段使用新进程。内存按五秒采样，可能低估瞬时峰值；全局提交余量反映采样时其他应用的占用。</p><img src="memory.png">
<p>OSQP 边界与状态依据：<a href="https://raw.githubusercontent.com/osqp/osqp/v0.6.3/include/constants.h">0.6.3 官方常量</a>、<a href="https://osqp.org/docs/release-0.6.3/interfaces/status_values.html">求解状态</a>；编译依据：<a href="https://gcc.gnu.org/onlinedocs/gcc/Common-Function-Attributes.html">GCC 函数属性</a>。</p>
<p>可复算文件：<a href="summary.json">原始摘要</a>、<a href="prefix_metrics.csv">前缀指标 CSV</a>、<a href="memory_metrics.csv">内存指标 CSV</a>。旧开发结果与中断记录保留在 history/ch5-v003-development-before-repair。</p></html>'''
    (OUT / 'index.html').write_text(page, 'utf-8')
    print(f'Repair acceptance report: {OUT / "index.html"}')


if __name__ == '__main__':main()
