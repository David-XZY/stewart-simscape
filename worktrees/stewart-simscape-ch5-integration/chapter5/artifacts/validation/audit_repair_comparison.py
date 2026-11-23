"""Recompute matched-prefix solver and candidate statistics from raw MAT files."""
from collections import Counter
from pathlib import Path
import csv
import hashlib
import html
import json
import numpy as np
from scipy.io import loadmat

ART = Path(__file__).resolve().parent.parent
ROOT = ART.parent


def get(value, name):
    return value[name] if isinstance(value, dict) else getattr(value, name)


def sequence(value):
    return np.asarray(value, dtype=object).reshape(-1)


def audit(path, calls, label):
    data = loadmat(path, variable_names=['records', 'predictions'], simplify_cells=True)
    records, predictions = sequence(data['records'])[:calls], sequence(data['predictions'])[:calls]
    assert len(records) == len(predictions) == calls
    row = dict(label=label, control_calls=calls, raw_file=str(path), raw_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    row['fallbacks'] = sum(bool(get(get(r, 'diagnostic'), 'usedFallback')) for r in records)
    checks = [c for p in predictions for c in sequence(get(get(p, 'prediction'), 'checks'))]
    row['total_candidates'] = len(checks)
    row['accepted_candidates'] = sum(bool(get(c, 'accepted')) for c in checks)
    row['controller_median_seconds'] = float(np.median([get(r, 'controller_seconds') for r in records]))
    for name in ['denoise_attempts', 'final_attempts']:
        attempts = [a for p in predictions for a in sequence(get(get(p, 'prediction'), name))]
        row[name] = dict(calls=len(attempts), non_success=sum(get(a, 'status') == 'exception' or not bool(get(a, 'solver_success')) for a in attempts), statuses=dict(Counter(str(get(a, 'status')) for a in attempts)))
    row['initial_q'] = np.asarray(get(records[0], 'q')).tolist()
    row['initial_goal'] = np.asarray(get(records[0], 'goal')).tolist()
    return row


def main():
    acceptance = json.loads((ART/'validation'/'repair_acceptance.json').read_text('utf-8'))
    assert acceptance['passed'], 'All independent repair acceptance phases must finish first.'
    identity = json.loads((ART/'runtime_source_identity.json').read_text('utf-8'))
    prefix = identity['diffusion_source_sha256'][:8]
    cases = [
        ('ideal-force', 51, ROOT/'history'/'ch5-v003-development-before-repair'/'development'/'development_002_ideal_force_diffusion_5101.mat', f'repair_{prefix}_002_ideal_force_1000ms_original.mat'),
        ('nonideal-force', 17, ART/'validation'/'repair_0f0f9173_002_nonideal_force_320ms_original.mat', f'repair_{prefix}_002_nonideal_force_320ms_original.mat')]
    pairs = []
    for mode, calls, old, new in cases:
        before = audit(old, calls, 'original-v003' if mode == 'ideal-force' else 'intermediate-v004-before-interval-constraints')
        after = audit(ART/'validation'/new, calls, 'current-v006')
        assert np.max(np.abs(np.asarray(before['initial_q'])-after['initial_q'])) < 1e-9
        assert np.max(np.abs(np.asarray(before['initial_goal'])-after['initial_goal'])) < 1e-9
        report = json.loads((ART/'validation'/new.replace('.mat', '_report.json')).read_text('utf-8'))
        assert report['passed'] and report['diffusion_source_sha256'] == identity['diffusion_source_sha256']
        after['actual_constraints_valid'] = report['all_constraints_valid']
        pairs.append(dict(mode=mode, before=before, after=after))
    out = ART/'repair_report';out.mkdir(exist_ok=True)
    summary = dict(diffusion_source_sha256=identity['diffusion_source_sha256'], seed=5101, pairs=pairs,
                   failed_phase_attempts=acceptance.get('failed_phase_attempts', []), recovery=acceptance.get('recovery'),
                   orchestration_amendment=acceptance.get('orchestration_amendment'),
                   boundary='Matched independent prefixes only. Nonideal before is intermediate v004, not the original v003. No full-task or formal superiority conclusion.')
    (out/'before_after.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), 'utf-8')
    fields = ['mode', 'stage', 'control_calls', 'fallbacks', 'total_candidates', 'accepted_candidates', 'denoise_calls', 'denoise_non_success', 'final_calls', 'final_non_success', 'controller_median_seconds']
    rows = []
    for pair in pairs:
        for stage in ['before', 'after']:
            value = pair[stage]
            rows.append(dict(mode=pair['mode'], stage=value['label'], **{k:value[k] for k in ['control_calls','fallbacks','total_candidates','accepted_candidates','controller_median_seconds']}, denoise_calls=value['denoise_attempts']['calls'], denoise_non_success=value['denoise_attempts']['non_success'], final_calls=value['final_attempts']['calls'], final_non_success=value['final_attempts']['non_success']))
    with (out/'before_after.csv').open('w', newline='', encoding='utf-8-sig') as handle:
        writer=csv.DictWriter(handle, fieldnames=fields);writer.writeheader();writer.writerows(rows)
    headers=''.join(f'<th>{html.escape(k)}</th>' for k in fields)
    cells=''.join('<tr>'+''.join(f'<td>{html.escape(str(row[k]))}</td>' for k in fields)+'</tr>' for row in rows)
    page=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>候选与求解状态修复对照</title><style>body{{font:16px system-ui;margin:32px;line-height:1.7}}table{{border-collapse:collapse;font-size:13px}}td,th{{border:1px solid #ccc;padding:6px}}</style><h1>候选与求解状态修复对照</h1><p>同一开发任务、同一起始位姿与目标、同一个训练模型（种子 5101）。理想执行器比较原 v003 与当前 v006；非理想执行器比较遗漏内部过渡约束的中间 v004 与当前 v006。</p><p>候选通过率、控制回退、求解器收敛分别计算。有限迭代结果只在完整动力学和物理约束检查通过后才可执行；未收敛状态保留，不改写为成功。</p><table><thead><tr>{headers}</tr></thead><tbody>{cells}</tbody></table><p>修复后的两段实际 Simscape 运动均通过硬约束检查。结果只覆盖这些独立前缀，不代表完整到达率或正式对比结论。CPU 求解耗时仍高，不能解释为实时部署性能。</p><p><a href="before_after.json">原始文件身份、全部状态与复算摘要</a> · <a href="before_after.csv">CSV</a> · <a href="index.html">完整修复检查报告</a></p></html>'''
    page=page.replace('<p>修复后的两段', '<h2>非理想执行器实际运动</h2><img src="nonideal_prefix_before_after.png" alt="修复前后实际运动" style="max-width:100%"><p><a href="nonideal_actual_timeseries.csv">实际运动时序 CSV</a> · <a href="nonideal_actual_comparison.json">原始记录哈希与物理指标</a></p><p>修复后的两段')
    if acceptance.get('failed_phase_attempts'):
        page=page.replace('</html>', '<h2>保留的失败尝试</h2><p>首次推理接口检查中，编译包装进程意外终止；采样没有显示内存耗尽，终止原因尚未确认。同一份源码和相同编译参数随后在独立进程中重放成功，相关子进程均以 0 退出。恢复时未改变根目录源码、控制算法、模型或约束。失败尝试及恢复身份完整记录在 before_after.json 和 validation/repair_acceptance.json，日志保留在 history/ch5-v006-inference-compiler-interruption。</p></html>')
    if acceptance.get('orchestration_amendment'):
        page=page.replace('</html>', '<h2>运行入口修正</h2><p>恢复开发实验时发现 Windows 启动 MATLAB 丢失双引号，首项任务尚未进入仿真。只修正运行编排中的参数引号和失败阶段记录；控制源码、配置、模型与任务文件逐项核对不变。真实 Windows MATLAB 启动通道已验证开发/正式阶段、两种执行器和基线/模型种子的参数传递，未生成正式任务。原验收阶段使用的源码哈希与入口修正前后清单分别保留，不将旧阶段冒充为新入口下重跑。诊断见 launcher_arguments.mat，身份证明见 before_after.json 的 orchestration_amendment。</p></html>')
    (out/'before_after.html').write_text(page, 'utf-8')
    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
