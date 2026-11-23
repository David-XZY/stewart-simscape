"""Recompute the V23--V36 development results without launching MATLAB.

Requires all six V33 normal episodes and its separately labelled fault probe.
This report deliberately does not write the formal freeze or campaign status.
"""
from __future__ import annotations

import csv
import hashlib
import html
import json
import subprocess
import warnings
import zipfile
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scipy.io

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "chapter5" / "artifacts" / "reduced_probe"
OUT = BASE / "restructure_report_v33"
PRODUCTION_HASH = "a0735e0e37317afcf4107359797e9fbab71fcb886584beac318b42edbcb050d6"
REQUIRED = [
    "ch5ReducedControlV33", "ch5ReducedStepV33", "ch5ReducedSolveV24",
    "ch5ReducedV33Sfun", "ch5ReducedTerminalV23", "ch5ReducedWarmV23",
    "ch5ReducedAuditV23", "ch5ReducedJacobianV17", "ch5ReducedPriorV21",
    "ch5ReducedCollisionAuditV22", "ch5ReducedCheckRolloutV22",
    "ch5ReducedHoldAuditV22", "ch5ReducedRecoveryFast",
]
AUDITS = [
    "failure_replay_v23", "tail_audit_v23", "restructure_audit_v24",
    "projection_audit_v25", "sensitivity_audit_v26", "direction_audit_v27b",
    "support_audit_v28", "support_task_audit_v30", "candidate_audit_v31",
    "numerics_audit_v33c", "feedback_candidate_audit_v34", "options_audit_v36",
    "refactor_final_audit_v37",
]


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def root_hash():
    digest = hashlib.sha256()
    for path in sorted((ROOT / "chapter5").iterdir()):
        if path.suffix in {".m", ".py", ".json"}:
            digest.update(path.name.encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


def write_csv(name, rows):
    with (OUT / name).open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def sequence(value):
    return [value] if isinstance(value, dict) else list(value)


def q_ms(values):
    return (1000 * np.quantile(values, [0.5, 0.95, 1])).tolist()


def metric_row(version, path):
    d = read(path)
    m = d["metrics"]
    assert path.with_suffix(".mat").is_file(), f"Missing raw episode: {path}"
    return dict(
        version=version, task=d["task"], mode=d["mode"],
        fault_injection=d["fault_injection"], simulated_seconds=m["simulated_end_seconds"],
        arrival=m["arrival"], safe_success=m["safe_success"],
        constraints_valid=m["all_constraints_valid"], collision=m["collision"],
        final_position_mm=1000*m["final_position"],
        final_rotation_deg=float(np.rad2deg(m["final_rotation"])),
        completion_seconds=m["completion_seconds"],
        wall_seconds=d["wall_seconds"],
        refresh_median_ms=q_ms(d["refresh_seconds"])[0],
        refresh_p95_ms=q_ms(d["refresh_seconds"])[1],
        refresh_max_ms=q_ms(d["refresh_seconds"])[2],
        execution_p95_ms=q_ms(d["execution_seconds"])[1],
        refresh_misses=d["refresh_deadline_misses"], refresh_count=len(d["refresh_seconds"]),
        execution_misses=d["execution_deadline_misses"], execution_count=len(d["execution_seconds"]),
        qp_solves=m["predictive_solves"], qp_nonconvergence=m["predictive_solver_failures"],
        fallbacks=m["fallback_count"], minimum_clearance_mm=(1000*m["minimum_clearance"]
            if m["minimum_clearance"] is not None else None),
        raw_path=str(path.with_suffix(".mat").relative_to(ROOT)),
    )


def html_table(rows, fields):
    def cell(value):
        return f"{value:.3f}" if isinstance(value, float) else str(value)
    return ("<table><thead><tr>" + "".join(f"<th>{html.escape(k)}</th>" for k in fields)
            + "</tr></thead><tbody>" + "".join("<tr>" + "".join(
                f"<td>{html.escape(cell(row[k]))}</td>" for k in fields) + "</tr>" for row in rows)
            + "</tbody></table>")


def main():
    assert root_hash() == PRODUCTION_HASH, "Registered production source changed."
    contract = read(BASE / "numerics_audit_v33c" / "summary.json")
    assert contract["accepted"] and not contract["error"]
    model_check = ROOT / "chapter5" / "artifacts" / "validation" / "model_compatibility.json"
    models = read(model_check)
    assert models["passed"] and {r["seed"] for r in models["models"]} == set(range(5101, 5106))
    source_hashes = {}
    for name in REQUIRED:
        path = ROOT / "chapter5" / "reduced" / f"{name}.m"
        source_hashes[name] = sha(path)
        assert source_hashes[name] == contract["identity"][f"{name}_m"], name

    expected = [(task, mode) for task in [1, 3, 4] for mode in ["ideal-force", "nonideal-force"]]
    episodes, raw_paths = [], []
    for version in ["v22", "v33"]:
        for task, mode in expected:
            path = BASE / f"closed_generic_{version}" / f"{task:03d}_{mode.replace('-', '_')}.json"
            row = metric_row(version, path)
            assert row["safe_success"] and row["simulated_seconds"] == 15
            assert not read(path)["simulation_error"]
            if version == "v33":
                for name, digest in source_hashes.items():
                    assert read(path)["code_identity"][f"{name}_m"] == digest
            episodes.append(row)
            raw_paths.extend([path, path.with_suffix(".mat")])
    fault_path = BASE / "recovery_generic_v33" / "001_nonideal_force.json"
    fault = metric_row("v33", fault_path)
    assert fault["fault_injection"] and fault["safe_success"] and fault["simulated_seconds"] == 15
    assert not read(fault_path)["simulation_error"]
    for name, digest in source_hashes.items():
        assert read(fault_path)["code_identity"][f"{name}_m"] == digest
    raw_paths.extend([fault_path, fault_path.with_suffix(".mat")])
    failed_path = BASE / "closed_generic_v28" / "004_nonideal_force.json"
    failed = metric_row("v28", failed_path)
    assert not failed["arrival"]
    raw_paths.extend([failed_path, failed_path.with_suffix(".mat")])

    audits = {name: read(BASE / name / "summary.json") for name in AUDITS}
    assert audits["refactor_final_audit_v37"]["accepted"]
    raw_paths.extend(BASE / name / suffix for name in AUDITS for suffix in ["summary.json", "audit.mat"])
    assert audits["tail_audit_v23"]["accepted"]
    qp_rows = []
    for row in audits["restructure_audit_v24"]["solver_rows"]:
        old, new = row["old_solver"], row["new_solver"]
        assert row["accepted"] and new["original_linear_feasible"] and new["solver_success"]
        qp_rows.append(dict(mode=row["mode"], time=row["time"], old_flag=old["exitflag"],
                            new_flag=new["exitflag"], old_iterations=old["iterations"],
                            new_iterations=new["iterations"], old_native_ms=1000*old["solver_seconds"],
                            new_native_ms=1000*new["solver_seconds"], new_total_ms=1000*new["total_seconds"],
                            stationarity=new["relative_stationarity"],
                            inequality_violation=new["original_inequality_violation"],
                            equality_error=new["original_equality_error"]))
    tail_rows = []
    for row in audits["tail_audit_v23"]["rows"]:
        if "old_seconds" not in row:
            continue
        old, new = np.median(row["old_seconds"]), np.median(row["new_seconds"])
        tail_rows.append(dict(mode=row["mode"], variant=row["variant"], old_median_ms=1000*old,
                              new_median_ms=1000*new, reduction_percent=100*(1-new/old),
                              maximum_numeric_error=max(row["errors"]), accepted=row["accepted"]))
    raw_candidates = audits["candidate_audit_v31"]["rows"]
    feedback = audits["feedback_candidate_audit_v34"]["rows"]
    candidate_counts = [dict(variant="raw", blend=1, valid=sum(c["check"]["accepted"] for r in raw_candidates for c in r["candidates"]),
                             total=sum(len(r["candidates"]) for r in raw_candidates), improving_groups=sum(
                                 r["best_valid_cost"] is not None and r["best_valid_cost"] < r["warm_cost"] for r in raw_candidates),
                             groups=len(raw_candidates))]
    for blend in [1, .1, .01]:
        rows = [r for r in feedback if r["blend"] == blend]
        candidate_counts.append(dict(variant="feedback", blend=blend,
            valid=sum(c["check"]["accepted"] for r in rows for c in r["candidates"]),
            total=sum(len(r["candidates"]) for r in rows), improving_groups=sum(
                r["best_valid_cost"] is not None and r["best_valid_cost"] < r["warm_cost"] for r in rows), groups=len(rows)))
    assert candidate_counts[0]["valid"] == 0 and candidate_counts[0]["total"] == 60
    assert sum(c["valid"] for c in candidate_counts[1:]) == 72
    assert sum(c["improving_groups"] for c in candidate_counts) == 0

    OUT.mkdir(exist_ok=True)
    write_csv("development_episodes.csv", episodes + [fault, failed])
    write_csv("frozen_qp_cases.csv", qp_rows)
    write_csv("terminal_microbench.csv", tail_rows)
    write_csv("candidate_admission.csv", candidate_counts)
    paired = []
    for old, new in zip(episodes[:6], episodes[6:]):
        paired.append(dict(task=new["task"], mode=new["mode"],
            old_p95_ms=old["refresh_p95_ms"], new_p95_ms=new["refresh_p95_ms"],
            old_wall_seconds=old["wall_seconds"], new_wall_seconds=new["wall_seconds"],
            old_qp_nonconvergence=old["qp_nonconvergence"], new_qp_nonconvergence=new["qp_nonconvergence"],
            old_fallbacks=old["fallbacks"], new_fallbacks=new["fallbacks"]))
    write_csv("paired_development.csv", paired)

    phase_rows = []
    warnings.filterwarnings("ignore", category=UserWarning, module="scipy")
    for version in ["v22", "v33"]:
        for task, mode in expected:
            path = BASE / f"closed_generic_{version}" / f"{task:03d}_{mode.replace('-', '_')}.mat"
            loaded = scipy.io.loadmat(path, variable_names=["predictions"], simplify_cells=True)
            refreshes = [p["prediction"] for p in sequence(loaded["predictions"]) if p["prediction"]["refresh"]]
            for name in refreshes[0]["phases"]:
                median, p95, maximum = q_ms([p["phases"][name] for p in refreshes])
                phase_rows.append(dict(version=version, task=task, mode=mode, phase=name,
                                       median_ms=median, p95_ms=p95, max_ms=maximum))
            del loaded, refreshes
    write_csv("controller_phases.csv", phase_rows)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    labels = [f"T{r['task']} {r['mode'].split('-')[0]}" for r in paired]
    x = np.arange(len(labels))
    for ax, key, ylabel in [(axes[0], "p95_ms", "Refresh P95 (ms)"), (axes[1], "wall_seconds", "15 s simulation wall time (s)")]:
        ax.bar(x-.18, [r[f"old_{key}"] for r in paired], .36, label="V22")
        ax.bar(x+.18, [r[f"new_{key}"] for r in paired], .36, label="V33")
        ax.set_xticks(x, labels, rotation=35, ha="right"); ax.set_ylabel(ylabel); ax.legend()
    axes[0].axhline(100, color="black", linestyle="--", linewidth=1)
    fig.tight_layout(); fig.savefig(OUT / "development_timing.png", dpi=170); plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    for version in ["v22", "v28", "v33"]:
        loaded = scipy.io.loadmat(BASE / f"closed_generic_{version}" / "004_nonideal_force.mat",
                                  variable_names=["records"], simplify_cells=True)
        records = sequence(loaded["records"])
        ax.plot([r["time"] for r in records], [1000*np.linalg.norm(r["x"][:3]-r["goal"][:3]) for r in records], label=version.upper())
        del loaded, records
    ax.axhline(5, color="black", linestyle="--", linewidth=1)
    ax.set(xlabel="Simulation time (s)", ylabel="Position error (mm)"); ax.legend()
    fig.tight_layout(); fig.savefig(OUT / "moving_position_error.png", dpi=170); plt.close(fig)

    raw_paths += sorted((ROOT / "chapter5" / "reduced").glob("*.m"))
    raw_paths.append(Path(__file__).resolve())
    raw_paths.append(model_check)
    raw_paths.append(ROOT / "chapter5" / "artifacts" / "teacher" / "dataset_manifest.json")
    for model in models["models"]:
        path = ROOT / "chapter5" / "artifacts" / "models" / f"seed_{model['seed']}.mat"
        assert sha(path) == model["matlab_sha256"]
        raw_paths.append(path)
    source_archive = OUT / "source_snapshot_v33.zip"
    tracked = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT).decode("utf-8").split("\0")
    code_extensions = {".m", ".py", ".json", ".slx", ".mdl", ".ps1", ".toml", ".yaml", ".yml", ".c", ".h", ".lock"}
    archive_paths = {ROOT / name for name in tracked if name and
                     (Path(name).suffix in code_extensions or Path(name).name == "README.md")}
    archive_paths.update((ROOT / "chapter5" / "reduced").glob("*.m"))
    archive_paths.update((ROOT / "chapter5" / "reduced").glob("*.py"))
    archive_paths.add(ROOT / "chapter5" / "reduced" / "README.md")
    with zipfile.ZipFile(source_archive, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(archive_paths):
            if path.is_file():
                archive.write(path, str(path.relative_to(ROOT)))
    raw_paths.append(source_archive)
    manifest = {str(path.relative_to(ROOT)): dict(sha256=sha(path), bytes=path.stat().st_size) for path in raw_paths}
    assert not (ROOT / "chapter5" / "artifacts" / "freeze.json").exists()
    totals = {v: {k: sum(r[k] for r in episodes if r["version"] == v) for k in [
        "refresh_misses", "refresh_count", "execution_misses", "execution_count", "qp_nonconvergence", "qp_solves", "fallbacks"]} for v in ["v22", "v33"]}
    summary = dict(generated_utc=datetime.now(timezone.utc).isoformat(), selected_development_revision="v33",
        development_regression_passed=True, formal_runs_completed=0, formal_runs_required=1440,
        production_source_sha256=PRODUCTION_HASH, selected_source_hashes=source_hashes,
        trained_model_identity=models["models"],
        episodes=episodes, fault_probe=fault, failed_support_closed_loop=failed,
        paired_development=paired, totals=totals, frozen_qp_cases=qp_rows,
        terminal_microbench=tail_rows, candidates=candidate_counts,
        memory_snapshot=audits["refactor_final_audit_v37"]["memory_snapshot"],
        executed_force_replays=audits["feedback_candidate_audit_v34"]["executed_force_replays"],
        boundary="Three development tasks, two modes, seed 5101; no baseline superiority or real-time acceptance.")
    (OUT / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    (OUT / "artifact_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    inventory = []
    for name in AUDITS + ["direction_audit_v27", "numerics_audit_v33", "numerics_audit_v33b"]:
        path = BASE / name / "summary.json"
        value = read(path)
        inventory.append(dict(name=name, completed=value.get("completed"),
            aggregate_accepted=value.get("accepted"), error=value.get("error", ""),
            evidence=str(path.relative_to(ROOT))))
    (OUT / "trial_inventory.json").write_text(json.dumps(inventory, ensure_ascii=False, indent=2), encoding="utf-8")
    table = html_table(episodes + [fault, failed], ["version", "task", "mode", "fault_injection", "safe_success",
        "final_position_mm", "wall_seconds", "refresh_p95_ms", "execution_p95_ms", "qp_nonconvergence", "fallbacks"])
    page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>规划控制重构开发实验 V33</title>
<style>body{{font:16px system-ui;max-width:1200px;margin:30px auto;padding:0 20px;line-height:1.7;color:#202b38}}
table{{border-collapse:collapse;font-size:12px;display:block;overflow:auto}}th,td{{border:1px solid #ccd5df;padding:6px}}img{{max-width:100%}}code{{overflow-wrap:anywhere}}.note{{background:#edf3f8;padding:15px}}</style>
<h1>规划控制重构：已完成的开发实验与结论</h1>
<p class="note">V33 的六次正常闭环和一次故障注入均完成 15 s 仿真并安全到达。它修复了本组 QP 不收敛问题，但未取得整体速度提升，也未通过 10 Hz 规划验收。正式对比仍为 0/1440，未冻结实验。本报告比较开发版本，不能替代 IHSID＋力前馈 LQI＋QP 与新方法的正式比较。</p>
<h2>保留的修改</h2><p>控制决策、代价、硬约束和执行逻辑沿用 V22。终端 Newton 修正仅重算最后 10 个区间，复用固定前 50 区间；三种输入变体、两种执行器及两次完整候选验收的数值一致性检查通过。六组微基准的中位耗时降低 {min(r['reduction_percent'] for r in tail_rows):.1f}–{max(r['reduction_percent'] for r in tail_rows):.1f}%，只表示终端模块。</p>
<p>QP 在 Hessian 白化后增加正的行缩放，保持相同优化问题，并在原始单位检查可行性、驻点残差和互补性。八个冻结矩阵案例均通过独立检查；六次正常闭环中，不收敛从 {totals['v22']['qp_nonconvergence']} 降到 {totals['v33']['qp_nonconvergence']}，回退仍为 {totals['v33']['fallbacks']}。一次强制拒绝探针单独保存，不能作为新增独立任务样本。</p>
<h2>闭环与计算结果</h2>{table}<img src="development_timing.png" alt="Paired development timing">
<p>正常运行刷新超出 100 ms：V22 为 {totals['v22']['refresh_misses']}/{totals['v22']['refresh_count']}，V33 为 {totals['v33']['refresh_misses']}/{totals['v33']['refresh_count']}；执行超出 20 ms：V33 为 {totals['v33']['execution_misses']}/{totals['v33']['execution_count']}。这是离线同步求解，仿真时间不等于实际部署时间。</p>
<p>计时来自完整控制调用；模型更新、冷编译、磁盘日志及离线指标审计另存。MATLAB PID 38888 持续复用，单次任务只重置物理状态和控制历史。V22 与 V33 使用相同开发任务，但测量顺序未随机、没有重复测量，不能作统计显著性或普遍速度提升结论。各阶段的中位数和 P95 不具有可加性。</p>
<h2>失败机制及未采用的尝试</h2><p>0.6 s 的原始候选在预测终点违反碰撞裕度约 0.11 mm；向已经不可行的初值回缩无法恢复可行。新鲜 Jacobian、普通第二次 QP、纯可行投影都未解决。定向几何检查中，QP 预测裕度 15.7 mm，完整展开约 15.27 mm，而位姿预测误差约 8×10⁻⁸（位置以 m、姿态分量以 rad）。证据指向圆柱支撑函数在轴线接近平面法向时的强曲率。这里的裕度是保守碰撞证书，不能当作精确实体净空。</p>
<p>保守支撑上图约束使四个局部候选通过完整检查，但 V28 的 15 s 闭环最终误差 {failed['final_position_mm']:.3f} mm、姿态误差 {failed['final_rotation_deg']:.3f}°，零回退、零 QP 失败仍未到达；不采用该重构。把该约束加入常规任务 QP，在五个停滞状态上也未降低完整任务代价。扩大 Jacobian 缓存虽提速，却超过预设的 1% 加权误差门槛；严格分支感知缓存通过误差检查，但收益有限。求解选项持久缓存得到相同解，八组微计时涨跌不一，因此没有声称它解决了总耗时。</p>
<img src="moving_position_error.png" alt="Moving-obstacle position error including failed revision">
<h2>扩散候选的有效性</h2><p>在 V28 保存的非理想执行器运动障碍状态 0.6、9.3、13 s，对全部五个训练种子分别生成四个真实候选。60 个原始力增量候选无一通过完整物理与终端验收；部分存在明显机械速度超限，不只是终端容差问题。内部反馈展开与缩幅后，180 个候选中 72 个通过，但 45 组均未得到比当前可行迭代更低的代价。</p>
{html_table(candidate_counts, ['variant', 'blend', 'valid', 'total', 'improving_groups', 'groups'])}
<p>此结果仅针对这些保存状态，不是五个模型的总体任务成功率。已执行力指令的六个 1.2 s 预测回放中，最大位置偏差约 0.36–0.74 mm、旋转偏差约 0.095–0.303°；有模型误差，但这些证据不足以把原始生成候选的大幅失效归因于某一个训练或模型问题。当前扩散主要以先验代价进入优化，尚未证明提供有效绕行或任务收益。</p>
<h2>结论与继续工作的依据</h2><p>可以保留精确终端前缀复用与等价 QP 行缩放，作为已通过本组开发回归的数值版本。保守几何修复与跨拒绝保留初值不能直接启用。下一阶段的主要问题是：碰撞局部求解兼顾可行与任务进展；生成候选在动力学下形成可行且有用的控制序列；非理想执行器导数、矩阵装配及完整候选检查满足计算预算。继续堆叠 QP、增加候选或放宽硬约束没有本轮证据支持。</p>
<p>安全结论仅覆盖审计采样网格及本组开发任务；没有连续时间安全、递归可行或整体稳定性证明。正式入口仍保持原版，未据开发结果挑选模型或正式工况。</p>
<h2>复算与审计</h2><p><a href="summary.json">汇总 JSON</a> · <a href="development_episodes.csv">逐次闭环 CSV</a> · <a href="paired_development.csv">开发版本配对 CSV</a> · <a href="controller_phases.csv">各阶段耗时</a> · <a href="frozen_qp_cases.csv">冻结 QP 案例</a> · <a href="terminal_microbench.csv">终端微基准</a> · <a href="candidate_admission.csv">候选验收</a> · <a href="trial_inventory.json">成功与失败记录</a> · <a href="artifact_manifest.json">原始 MAT/JSON 与源码哈希</a></p>
<p><a href="source_snapshot_v33.zip">保留原始字节与行尾的源码/模型快照</a>包含工程代码和 Simulink 模型；训练权重、教师数据及原始运行记录通过清单追溯至本工作区。恢复快照后仍需本机 MATLAB、工具箱与已记录的 Python/CasADi 依赖。</p>
<p>源码守卫：<code>{PRODUCTION_HASH}</code>。复算命令：<code>chapter5/.venv/Scripts/python.exe chapter5/reduced/restructure_report.py</code>。本报告生成过程不启动 MATLAB，不修改模型、训练权重或正式实验状态。</p></html>'''
    (OUT / "index.html").write_text(page, encoding="utf-8")
    print(json.dumps(dict(report=str(OUT / "index.html"), totals=totals, formal_completed=0), ensure_ascii=False))


if __name__ == "__main__":
    main()
