"""Keep development repair evidence current; never label it formal acceptance."""
from pathlib import Path
from datetime import datetime, timezone
import html
import json
import time

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT / 'chapter5/artifacts/control_force_v62'
OUT = BASE / 'teacher_failure_v83'
IPC = ROOT / 'chapter5/artifacts/performance_probe'


def read(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8-sig'))
    except (OSError, json.JSONDecodeError):
        return {}


def refresh():
    replay = read(OUT / 'replay.json')
    replan = read(OUT / 'replan_probe.json')
    restart = read(OUT / 'restart_audit_v86.json')
    public = read(OUT / 'safety_replay_v87.json')
    exact = read(OUT / 'exact_probe_v85.json')
    full = read(BASE / 'teacher_repair_v86/summary.json')
    worker = read(IPC / 'worker_status.json')
    status = dict(timestamp_utc=datetime.now(timezone.utc).isoformat(), formal=False, formal_runs=0,
                  observation_replay_completed=bool(replay.get('completed')),
                  earlier_backup_plan_accepted=bool(replan.get('accepted')),
                  feasible_seed_qp_completed=bool(restart.get('completed')),
                  public_filter_replay_completed=bool(public.get('completed')),
                  full_repair_completed=bool(full.get('completed')),
                  full_repair_result_states=len(full.get('rows', [])),
                  full_repair_admitted=sum(bool(r.get('admitted')) for r in full.get('rows', [])),
                  moving_model_retrained=False, worker=worker)
    (OUT / 'progress.json').write_text(json.dumps(status, ensure_ascii=False, indent=2), encoding='utf-8')
    rows = []
    for row in full.get('rows', []):
        m = row.get('metrics', {})
        cells = [row['task'], row['mode'], row['status'], str(row.get('admitted', False)),
                 str(m.get('fallback_count', '')), str(m.get('minimum_clearance', '')),
                 str(row.get('restart_count', '')), str(row.get('wall_seconds', ''))]
        rows.append('<tr>' + ''.join('<td>' + html.escape(c) + '</td>' for c in cells) + '</tr>')
    exact_text = ('尚未结束；已观察到前两组初值分别耗时217.5、214.5秒且失败。'
                  if not exact else f"四组初值已结束，是否通过完整检查：{exact.get('accepted')}，总耗时：{exact.get('seconds', 0):.1f}秒。")
    text = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>第五章动态教师失败排查与修复</title>
<style>body{{font:16px/1.65 system-ui,sans-serif;max-width:1100px;margin:32px auto;padding:0 24px;color:#203047}}
table{{border-collapse:collapse;width:100%;font-size:14px}}td,th{{border:1px solid #ccd3dd;padding:8px}}img{{max-width:100%}}
code{{background:#eef1f5;padding:2px 5px}}.status{{background:#eaf1fa;padding:16px}}</style>
<h1>动态教师失败排查与修复：开发验证</h1>
<p class="status">更新时间：{html.escape(status['timestamp_utc'])}。正式实验0/1440；移动数据尚未重训练。
原批次18个结果状态、11个有效运行、7个失败。修复版完整复验：{status['full_repair_result_states']}/3个结果状态，
{status['full_repair_admitted']}个通过训练准入。当前MATLAB作业：{html.escape(worker.get('id',''))}，{html.escape(worker.get('state',''))}。</p>
<h2>已核实的两类失败</h2>
<p>六个转向工况在6.18秒失去碰撞psi1条件，6.26秒首次出现QP/保持区间失败。原目标到达后的保留检查只覆盖0.2秒。
使用相同已观测状态与恒速度预测，把目标保持检查扩展到1.2秒，六个记录均在5.48秒被拒绝，提前0.78秒。
这是保存观测的离线重放结果，还需闭环验证。提前重规划找到了通过原物理后验检查的退避轨迹，但目标规划的四组初值都耗尽迭代。</p>
<p>train_016非理想执行器在13.78秒的失败来自机械保持区间检查；当前碰撞h与psi1仍有效。
同一状态下，平衡力和三组邻近指令能通过全部原检查，因此原局部QP投影失败不等于物理可行集合为空。
新增修正仅从完整检查通过的种子出发，求解原七变量CLF-CBF-QP目标，采用逐步缩小的力信赖域，并再次执行完整非线性保持与执行器瞬态检查。
物理界限、1%预测加速度裕度、输入变化率和碰撞条件均保留，CBF没有新增松弛变量。</p>
<p>QP修正独立验证完成：{status['feasible_seed_qp_completed']}；完整安全接口重放完成：{status['public_filter_replay_completed']}。</p>
<img src="failure_timeline.png" alt="原失败运行的碰撞裕度与psi1时序">
<h2>求解方式试验与内存边界</h2>
<p>精确Hessian整图JIT编译失败，并留下编译进程；已保存错误和进程身份，清理确认属于本次编译的进程，保持MATLAB进程20004。
不整图编译的精确Hessian试验：{html.escape(exact_text)} 修复版仍使用原规划求解方式，此试验未被采用。</p>
<h2>三个固定失败条件的完整Simscape复验</h2>
<table><thead><tr><th>任务</th><th>执行器</th><th>状态</th><th>有效准入</th><th>回退</th><th>最小实际净空/m</th><th>可行种子修正</th><th>墙钟/s</th></tr></thead>
<tbody>{''.join(rows)}</tbody></table>
<p>任务固定为train_016非理想执行器、train_012理想执行器、train_012非理想执行器。
通过准入必须完整仿真15秒、到达且满足全部实际硬约束，没有仿真错误。失败、未到达和停止记录全部保留。
模拟时间与计算墙钟分别报告；这些结果不构成10Hz、全局稳定性、递归可行性或连续时间安全证明。</p>
<h2>依据与可复算证据</h2>
<p><a href="https://arxiv.org/abs/2403.19122">动态障碍规划与高阶CBF</a>区分初始可行条件、预测模型及约束可行性。
<a href="https://coin-or.github.io/Ipopt/OPTIONS.html">IPOPT官方选项</a>说明精确Hessian、拟牛顿和终止条件；选择哪种计算方式需要测量。
<a href="https://learn.microsoft.com/en-us/windows/win32/procthread/job-objects">Windows Job Objects</a>说明进程生命周期与子进程约束。
这些文献说明设计依据，不直接证明Stewart组合方法的性质。</p>
<p><a href="failure_evidence.json">原失败审计</a> · <a href="replay.json">预测检查重放</a> ·
<a href="replan_probe.json">提前重规划</a> · <a href="hold_seeds.json">可行种子检查</a> ·
<a href="restart_audit_v86.json">QP修正验证</a> · <a href="safety_replay_v87.json">完整安全接口重放</a> ·
<a href="../teacher_repair_v86/identity.json">冻结源码与原466项哈希</a> · <a href="../teacher_repair_v86/summary.json">完整复验原始汇总</a></p>
</html>'''
    (OUT / 'index.html').write_text(text, encoding='utf-8')
    return status


def main():
    deadline = time.monotonic() + 10800
    previous = None
    while time.monotonic() < deadline:
        status = refresh()
        notice = (status['worker'].get('id'), status['worker'].get('state'),
                  status['public_filter_replay_completed'], status['full_repair_result_states'])
        if notice != previous:
            print('Repair progress:', notice, flush=True)
            previous = notice
        if status['full_repair_completed']:
            return
        final = read(BASE / 'teacher_repair_v86_full_status.json')
        if final and final[-1]['result'].get('state') == 'failed':
            print('Pipeline failed; preserved all records.', flush=True)
            return
        time.sleep(5)
    print('Report watcher time limit; result states remain explicit.', flush=True)


if __name__ == '__main__':
    main()
