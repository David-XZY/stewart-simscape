"""Recompute literature-guided V38--V54 diagnostics; retain failed trials.

No MATLAB launch, training, formal freeze, task filtering or model selection.
The original V33 report is never overwritten.
"""
from __future__ import annotations

import csv
import hashlib
import html
import json
import subprocess
import sys
import zipfile
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat
import scipy

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "chapter5" / "artifacts" / "reduced_probe"
OUT = BASE / "literature_report_v54"
PRODUCTION_HASH = "a0735e0e37317afcf4107359797e9fbab71fcb886584beac318b42edbcb050d6"
AUDITS = ["thread_map_audit_v38", "teacher_candidate_audit_v39",
          "projected_prior_audit_v40", "hybrid_qp_audit_v41", "horizon_audit_v43",
          "horizon_audit_v43b", "horizon_audit_v45", "affine_prior_audit_v46",
          "blocking_audit_v48", "blocking_audit_v50", "literature_final_audit_v51"]
SOURCES = [
    dict(title="DPCC", url="https://arxiv.org/html/2412.09342v2",
         evidence="Reverse diffusion contains model/constraint projection; goal-independent projection can harm task progress.",
         use="V40 full-gate feasible-ray prototype and V46 limited affine projection, neither an implementation of the exact DPCC projection.",
         boundary="Published convex assumptions and low-dimensional velocity-input experiments do not prove this Stewart force controller safe or stable."),
    dict(title="Diffusion Policy", url="https://diffusion-policy.cs.columbia.edu/diffusion_policy_2023.pdf",
         evidence="Action representation and accumulated error affect closed-loop performance; normalization must agree with clipping.",
         use="V39 measures both force-increment error and the resulting cumulative force error.",
         boundary="Their position-action findings do not justify replacing the required force decisions with a pose tracker."),
    dict(title="CasADi map", url="https://web.casadi.org/docs/#map",
         evidence="Mapped functions support a thread backend with an explicit thread count.",
         use="V38 checks exact outputs and interleaved serial/2-thread/4-thread timings.",
         boundary="Thread support is not a speed guarantee for this native Windows workload."),
    dict(title="MathWorks quadprog", url="https://www.mathworks.com/help/optim/ug/quadprog.html",
         evidence="Native solver flags, iteration limits and numerical feasibility require separate interpretation.",
         use="V41 bounds the first unscaled solve and retains both first-attempt and scaled-retry flags.",
         boundary="A successful retry does not erase the failed first attempt or its cost."),
    dict(title="MathWorks horizons", url="https://www.mathworks.com/help/mpc/ug/choosing-sample-time-and-horizons.html",
         evidence="Prediction must anticipate constraints; shortening it trades performance against computation. Fewer independent moves reduce QP variables.",
         use="V45 tests H30 separately; V48 keeps H60 and coarsens the correction subspace.",
         boundary="Neither a median timing nor an unconstrained design recommendation proves constrained task completion."),
    dict(title="MathWorks variable blocking", url="https://www.mathworks.com/help/mpc/ug/manipulated-variable-blocking.html",
         evidence="Blocking groups independent moves; piecewise-constant controls can be restrictive in nonlinear path planning.",
         use="V48 changes correction blocks from 5 to 10 intervals; internal feedback still acts at every interval.",
         boundary="Our blocked feedback corrections are a Stewart adaptation, not an exact copy of the built-in MPC implementation or a feasibility theorem."),
    dict(title="SDEdit", url="https://arxiv.org/abs/2108.01073",
         evidence="A noised guide can initialize an intermediate diffusion time.",
         use="V40/V46 explore intermediate-time initialization from a known force sequence.",
         boundary="Image-editing evidence motivates this experiment only; it provides no robot-control guarantee."),
]


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def root_hash():
    digest = hashlib.sha256()
    for path in sorted((ROOT / "chapter5").iterdir()):
        if path.suffix in {".m", ".py", ".json"}:
            digest.update(path.name.encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


def number(value):
    return float("inf") if value is None else value


def csv_write(name, rows):
    assert rows
    with (OUT / name).open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def quantile_ms(values):
    return (1000 * np.quantile(values, [.5, .95, 1])).tolist()


def metric_row(version, path):
    d = read(path)
    m = d["metrics"]
    assert path.with_suffix(".mat").exists()
    return dict(version=version, task=d["task"], mode=d["mode"], fault=d["fault_injection"],
                horizon_seconds=d.get("prediction_horizon_seconds", 1.2),
                arrival=m["arrival"], constraints_valid=m["all_constraints_valid"],
                safe_success=m["safe_success"], simulated_seconds=m["simulated_end_seconds"],
                final_position_mm=1000*m["final_position"],
                final_rotation_deg=float(np.rad2deg(m["final_rotation"])),
                wall_seconds=d["wall_seconds"], build_seconds=d["build_seconds"],
                refresh_median_ms=quantile_ms(d["refresh_seconds"])[0],
                refresh_p95_ms=quantile_ms(d["refresh_seconds"])[1],
                refresh_max_ms=quantile_ms(d["refresh_seconds"])[2],
                execution_median_ms=quantile_ms(d["execution_seconds"])[0],
                execution_p95_ms=quantile_ms(d["execution_seconds"])[1],
                execution_max_ms=quantile_ms(d["execution_seconds"])[2],
                refresh_misses=d["refresh_deadline_misses"], refresh_count=len(d["refresh_seconds"]),
                execution_misses=d["execution_deadline_misses"], execution_count=len(d["execution_seconds"]),
                qp_solves=m["predictive_solves"], qp_failures=m["predictive_solver_failures"],
                fallbacks=m["fallback_count"], simulation_error=d["simulation_error"],
                evidence=str(path.relative_to(ROOT)))


def table(rows, fields):
    def cell(value):
        return f"{value:.3f}" if isinstance(value, float) else str(value)
    return ("<table><thead><tr>" + "".join(f"<th>{html.escape(k)}</th>" for k in fields)
            + "</tr></thead><tbody>" + "".join("<tr>" + "".join(
                f"<td>{html.escape(cell(row[k]))}</td>" for k in fields) + "</tr>" for row in rows)
            + "</tbody></table>")


def main():
    assert root_hash() == PRODUCTION_HASH, "Registered production source changed."
    assert not (ROOT / "chapter5" / "artifacts" / "freeze.json").exists()
    OUT.mkdir(exist_ok=True)
    audits = {name: read(BASE / name / "summary.json") for name in AUDITS}
    final = audits["literature_final_audit_v51"]
    assert final["accepted"], "Final source/artifact audit is required."
    assert audits["horizon_audit_v45"]["accepted"] and audits["blocking_audit_v48"]["accepted"]
    assert audits["teacher_candidate_audit_v39"]["completed"]
    assert audits["projected_prior_audit_v40"]["completed"]
    assert audits["affine_prior_audit_v46"]["completed"]
    models = read(ROOT / "chapter5" / "artifacts" / "validation" / "model_compatibility.json")
    assert models["passed"] and {m["seed"] for m in models["models"]} == set(range(5101, 5106))
    raw_paths = []
    for model in models["models"]:
        path = ROOT / "chapter5" / "artifacts" / "models" / f"seed_{model['seed']}.mat"
        assert sha(path) == model["matlab_sha256"]
        raw_paths.append(path)

    dataset_manifest = read(ROOT / "chapter5" / "artifacts" / "teacher" / "dataset_manifest.json")
    teacher_catalog = {row["file"]:row for row in dataset_manifest["raw_runs"]}
    teacher_rows, neural_rows = [], []
    for row in audits["teacher_candidate_audit_v39"]["rows"]:
        raw, repaired = row["teacher_raw_check"], row["teacher_repaired_check"]
        source = row["source"]
        catalog = teacher_catalog[source["run"]]
        # The export's teacher_source_sha256 is the baseline controller source
        # identity. The raw MAT file digest is a separate catalog.sha256 field.
        assert catalog["teacher_source_sha256"] == source["teacher_source_sha256"]
        teacher_rows.append(dict(mode=row["mode"], window=row["window_index"],
            source=source["run"], teacher_controller_source_sha256=source["teacher_source_sha256"],
            raw_mat_sha256=catalog["sha256"],
            time=row["source"]["first_time"], raw_physical_valid=bool(raw["finite"] and
                number(raw["inequality_violation"]) <= 1e-6 and number(raw["command_rate_violation"]) <= 1e-6),
            raw_full_valid=raw["accepted"], raw_terminal_ratio=raw["equality_tolerance_ratio"],
            repaired_full_valid=repaired["accepted"],
            model_max_position_error_mm=1000*row["teacher_maximum_position_prediction_error"]))
        teacher_path = ROOT / "chapter5" / "artifacts" / "teacher" / row["source"]["run"]
        if teacher_path not in raw_paths:
            assert teacher_path.is_file() and sha(teacher_path) == catalog["sha256"], "Teacher raw-file identity changed."
            raw_paths.append(teacher_path)
        for gen in row["generated"]:
            check = gen["check"]
            neural_rows.append(dict(mode=row["mode"], window=row["window_index"], seed=gen["seed"],
                candidate=gen["candidate"], finite=check["finite"],
                physical_valid=bool(check["finite"] and number(check["inequality_violation"]) <= 1e-6),
                full_valid=check["accepted"], increment_rms_error_N=gen["increment_rms_error"],
                force_rms_error_N=gen["force_rms_error"]))
    assert len(teacher_rows) == 24 and len(neural_rows) == 480
    csv_write("teacher_windows.csv", teacher_rows)
    csv_write("neural_candidates.csv", neural_rows)

    projection_rows = []
    for row in audits["projected_prior_audit_v40"]["rows"]:
        cs, info = row["candidates"], row["info"]
        projection_rows.append(dict(variant="full_gate_ray_v40", mode=row["mode"],
            sample=row["time"], seed=row["seed"], start_t=row["start_t"], anchor_accepted=row["anchor_accepted"],
            candidates=len(cs), raw_full_valid=None, repaired_full_valid=sum(c["check"]["accepted"] for c in cs),
            improving=sum(c["improved"] for c in cs), seconds=info["seconds"],
            projection_ms=None, maximum_affine_residual=None))
    skipped_anchors = []
    for row in audits["affine_prior_audit_v46"]["rows"]:
        if "candidates" not in row:
            skipped_anchors.append(dict(mode=row["mode"], window=row["window_index"], check=row["anchor_check"]))
            continue
        cs, info = row["candidates"], row["info"]
        projection_rows.append(dict(variant="affine_v46", mode=row["mode"],
            sample=row["window_index"], seed=row["seed"], start_t=row["start_t"], anchor_accepted=True,
            candidates=len(cs), raw_full_valid=sum(c["raw_check"]["accepted"] for c in cs),
            repaired_full_valid=sum(c["check"]["accepted"] for c in cs),
            improving=sum(c["check"]["accepted"] and c["cost"] < row["anchor_cost"]-1e-10 for c in cs),
            seconds=info["seconds"], projection_ms=1000*info["projection_seconds"],
            maximum_affine_residual=info["maximum_affine_residual"]))
    csv_write("projection_groups.csv", projection_rows)
    projection_summary = []
    for label in ["full_gate_ray_v40", "affine_v46"]:
        for t in sorted({r["start_t"] for r in projection_rows if r["variant"] == label}):
            rows = [r for r in projection_rows if r["variant"] == label and r["start_t"] == t]
            projection_summary.append(dict(variant=label, start_t=t, groups=len(rows),
                total=sum(r["candidates"] for r in rows), valid=sum(r["repaired_full_valid"] for r in rows),
                improving=sum(r["improving"] for r in rows),
                improving_groups=sum(r["improving"] > 0 for r in rows),
                median_ms=quantile_ms([r["seconds"] for r in rows])[0],
                p95_ms=quantile_ms([r["seconds"] for r in rows])[1]))
    csv_write("projection_summary.csv", projection_summary)

    microbench = []
    for row in audits["thread_map_audit_v38"]["rows"]:
        times = np.array(row["seconds"])
        for ci, threads in enumerate(row["threads"]):
            microbench.append(dict(variant="map_threads", mode=row["mode"], time=row["time"],
                setting=threads, median_ms=1000*float(np.median(times[:, ci])),
                numerical_error=row["relative_errors"][ci], accepted_scope=row["accepted"]))
    for row in audits["hybrid_qp_audit_v41"]["rows"]:
        times = np.array(row["seconds"])
        for ci, label in enumerate(["scaled_reference", "bounded_retry"]):
            microbench.append(dict(variant="qp_retry", mode=row["mode"], time=row["time"],
                setting=label, median_ms=1000*float(np.median(times[:, ci])),
                numerical_error=row["delta_difference"], accepted_scope=row["accepted"]))
    for audit_name, variant in [("blocking_audit_v48", "uniform_block10"), ("blocking_audit_v50", "nonuniform_middle_blocks")]:
        for row in audits[audit_name]["rows"]:
            for label, times in [("block5", row["old_seconds"]), (variant, row["new_seconds"])]:
                microbench.append(dict(variant=variant, mode=row["mode"], time=row["time"],
                    setting=label, median_ms=1000*float(np.median(times)),
                    numerical_error=max(row["embedding_errors"]), accepted_scope=row["accepted"]))
    csv_write("microbench.csv", microbench)

    episodes, episode_paths = [], []
    for version, directory in [("v33", "closed_generic_v33"), ("v45_H30", "closed_short_v45"),
                               ("v48_block10", "closed_blocked_v48"), ("v50_nonuniform", "closed_blocked_v50"),
                               ("v50_nonuniform_fault", "recovery_blocked_v50"),
                               ("v54_preheated", "closed_preheated_v54")]:
        for path in sorted((BASE / directory).glob("00*force.json")):
            episodes.append(metric_row(version, path));episode_paths.append(path)
    assert any(r["version"] == "v45_H30" and not r["arrival"] for r in episodes)
    csv_write("development_episodes.csv", episodes)
    phase_rows, failure_rows = [], []
    def cells(value):
        if isinstance(value, dict):return [value]
        return list(value)
    for episode, path in zip(episodes, episode_paths):
        if episode["version"] not in {"v33", "v50_nonuniform", "v54_preheated"}:
            continue
        data = loadmat(path.with_suffix(".mat"), simplify_cells=True, variable_names=["records", "predictions"])
        predictions = data["predictions"]
        refreshes = [p["prediction"] for p in predictions if p["prediction"]["refresh"]]
        for phase in refreshes[0]["phases"]:
            values = [p["phases"][phase] for p in refreshes]
            med, p95, maximum = quantile_ms(values)
            phase_rows.append(dict(version=episode["version"], task=episode["task"], mode=episode["mode"],
                                   phase=phase, median_ms=med, p95_ms=p95, max_ms=maximum))
        for p in predictions:
            for attempt in cells(p["prediction"]["denoise_attempts"]):
                if isinstance(attempt, dict) and not attempt["solver_success"]:
                    failure_rows.append(dict(version=episode["version"], task=episode["task"], mode=episode["mode"],
                        time=p["time"], exitflag=attempt["exitflag"], iterations=attempt["iterations"],
                        original_linear_feasible=attempt["original_linear_feasible"],
                        relative_stationarity=attempt["relative_stationarity"],
                        solver_ms=1000*attempt["solver_seconds"]))
        del data, predictions
    csv_write("controller_phases.csv", phase_rows)
    if failure_rows:csv_write("qp_failures.csv", failure_rows)
    blocked = [r for r in episodes if r["version"] == "v50_nonuniform"]
    blocked_fault = [r for r in episodes if r["version"] == "v50_nonuniform_fault"]
    admitted_blocking = len(blocked) == 6 and len(blocked_fault) == 1 and all(
        r["safe_success"] and r["simulated_seconds"] == 15 and not r["simulation_error"]
        for r in blocked + blocked_fault)
    before = loadmat(BASE / "closed_blocked_v50" / "004_nonideal_force.mat", simplify_cells=True, variable_names=["records"])["records"]
    after = loadmat(BASE / "closed_preheated_v54" / "004_nonideal_force.mat", simplify_cells=True, variable_names=["records"])["records"]
    assert len(before) == len(after) == 751
    warmup_consistency = {field:float(max(np.max(np.abs(a[field]-b[field])) for a,b in zip(before, after)))
                          for field in ["time", "x", "q", "u", "previous_u", "goal"]}
    (OUT / "warmup_consistency.json").write_text(json.dumps(warmup_consistency, indent=2), encoding="utf-8")
    del before, after

    fig, axes = plt.subplots(1, 2, figsize=(10, 3.7))
    for ax, field, title in zip(axes, ["increment_rms_error_N", "force_rms_error_N"],
                                ["Force increment error", "Cumulative force error"]):
        for mode in ["ideal-force", "nonideal-force"]:
            values = [r[field] for r in neural_rows if r["mode"] == mode]
            ax.hist(values, bins=25, alpha=.6, label=mode)
        ax.set(xlabel="RMS error (N)", ylabel="Candidate count", title=title)
        ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(OUT / "force_error.png", dpi=170);plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.2), sharex=True)
    for row, path in zip(episodes, episode_paths):
        if row["task"] != 4 or row["mode"] != "nonideal-force":
            continue
        data = loadmat(path.with_suffix(".mat"), simplify_cells=True, variable_names=["records"])
        records = data["records"]
        t = [r["time"] for r in records]
        error = [1000*np.linalg.norm(r["q"][:3]-r["goal"][:3]) for r in records]
        axes[0].plot(t, error, label=row["version"])
        axes[1].plot(t, [1000*r["controller_seconds"] for r in records], linewidth=.7, label=row["version"])
    axes[0].axhline(5, color="black", linestyle="--", linewidth=1)
    axes[1].axhline(100, color="black", linestyle="--", linewidth=1)
    axes[0].set(ylabel="Position error (mm)", title="Moving obstacle / nonideal actuator")
    axes[1].set(ylabel="Controller time (ms)", xlabel="Simulation time (s)")
    for ax in axes:ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(OUT / "moving_closed_loop.png", dpi=170);plt.close(fig)

    # Preserve exact source bytes, including native Windows line endings.
    tracked = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT).decode("utf-8").split("\0")
    extensions = {".m", ".py", ".json", ".slx", ".mdl", ".ps1", ".toml", ".yaml", ".yml", ".c", ".h", ".lock"}
    source_paths = {ROOT / p for p in tracked if p and (Path(p).suffix in extensions or Path(p).name == "README.md"
                   or Path(p).name.startswith("requirements"))}
    source_paths.update(p for p in (ROOT / "chapter5" / "reduced").iterdir() if p.suffix in {".m", ".py", ".md"})
    archive_path = OUT / "source_snapshot_v54.zip"
    with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(source_paths):
            if path.is_file():archive.write(path, str(path.relative_to(ROOT)))
    raw_paths.extend(source_paths)
    for name in AUDITS:
        raw_paths.extend([BASE / name / "summary.json", BASE / name / "audit.mat"])
    for path in episode_paths:raw_paths.extend([path, path.with_suffix(".mat")])
    raw_paths.extend([archive_path, ROOT / "chapter5" / "artifacts" / "validation" / "model_compatibility.json"])
    raw_paths.append(ROOT / "chapter5" / "artifacts" / "teacher" / "dataset_manifest.json")
    environment_path = OUT / "report_environment.json"
    environment_path.write_text(json.dumps(dict(python=sys.version, numpy=np.__version__, scipy=scipy.__version__,
        matplotlib=matplotlib.__version__, source_head=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT).decode().strip()),
        indent=2), encoding="utf-8")
    raw_paths.append(environment_path)
    manifest = {str(path.relative_to(ROOT)): dict(sha256=sha(path), bytes=path.stat().st_size)
                for path in sorted(set(raw_paths)) if path.is_file()}
    inventory = [dict(name=name, completed=d.get("completed"), accepted_scope=d.get("accepted"),
                      error=d.get("error", ""), evidence=str((BASE / name / "summary.json").relative_to(ROOT)))
                 for name, d in audits.items()]
    summary = dict(generated_utc=datetime.now(timezone.utc).isoformat(), production_source_sha256=PRODUCTION_HASH,
        formal_ready=False, formal_runs_completed=0, formal_runs_required=1440,
        v50_complete_development_regression=admitted_blocking,
        selected_development_revision="v33", evaluated_development_candidate="v50",
        hotpath_preheating_consistency=warmup_consistency, qp_failures=failure_rows,
        teacher_windows=len(teacher_rows), teacher_raw_physical_valid=sum(r["raw_physical_valid"] for r in teacher_rows),
        teacher_raw_full_valid=sum(r["raw_full_valid"] for r in teacher_rows),
        teacher_repaired_full_valid=sum(r["repaired_full_valid"] for r in teacher_rows),
        neural_candidates=len(neural_rows), neural_raw_full_valid=sum(r["full_valid"] for r in neural_rows),
        neural_raw_physical_valid=sum(r["physical_valid"] for r in neural_rows),
        increment_rms_quantiles_N=np.quantile([r["increment_rms_error_N"] for r in neural_rows], [.1,.5,.9]).tolist(),
        force_rms_quantiles_N=np.quantile([r["force_rms_error_N"] for r in neural_rows], [.1,.5,.9]).tolist(),
        projection_summary=projection_summary, skipped_affine_anchors=skipped_anchors,
        episodes=episodes, models_unchanged=True, literature=SOURCES, final_audit=final,
        boundary="Development only; diagnostic candidates are correlated within window/model; no formal success rate, statistical superiority, continuous safety or 10 Hz acceptance.")
    for name, value in [("summary.json", summary), ("artifact_manifest.json", manifest), ("trial_inventory.json", inventory)]:
        (OUT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    refs = "".join(f'<li><a href="{s["url"]}">{s["title"]}</a>：{html.escape(s["evidence"])} '
                   f'本轮用途：{html.escape(s["use"])} 边界：{html.escape(s["boundary"])}</li>' for s in SOURCES)
    block_statement = ("V50 完成六个正常开发闭环和一个独立故障探针，均安全到达；保留为计算压缩候选。QP 与耗时长尾尚未完整解决，当前模板继续使用 V33，尚未正式验收。"
                       if admitted_blocking else "V50 的完整开发回归未通过；保留失败与已运行记录，继续使用此前通过回归的 V33。")
    page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>文献驱动的规划控制排查 V38–V54</title>
<style>body{{font:16px system-ui;max-width:1200px;margin:30px auto;padding:0 20px;line-height:1.7;color:#202b38}}table{{border-collapse:collapse;font-size:12px;display:block;overflow:auto}}th,td{{border:1px solid #ccd5df;padding:6px}}img{{max-width:100%}}code{{overflow-wrap:anywhere}}.note{{background:#edf3f8;padding:15px}}</style>
<h1>规划控制一体化：文献驱动的排查与实际实验</h1>
<p class="note">本轮已执行候选诊断、去噪投影、线程与 QP 微基准，以及独立 Simscape 开发试验。{block_statement} 正式比较仍为 0/1440，未冻结配置。两次失败的短时域接口审计和后续未到达运行均保留，不能依据计算更快或零 QP 失败删除这些结果。</p>
<h2>主要新证据</h2><p>固定随机抽取的 24 个验证任务窗口中，教师原始指令的机械、碰撞与输入检查全部通过，但严格终端条件为 0/24；现有 Newton 终端修正使 23/24 可接受。原始教师截取窗口并不自然满足短时域静止终端，因此原始标签分布与完整候选验收存在差异。它不构成教师闭环失败的证据。</p>
<p>五个已完成训练的模型在这些窗口中各生成四个候选：480 个原始候选无一通过完整验收，机械/碰撞不等式也全部未通过。增量 RMS 误差的 10/50/90 分位为 {summary['increment_rms_quantiles_N'][0]:.3f}/{summary['increment_rms_quantiles_N'][1]:.3f}/{summary['increment_rms_quantiles_N'][2]:.3f} N，累加后的力序列误差为 {summary['force_rms_quantiles_N'][0]:.2f}/{summary['force_rms_quantiles_N'][1]:.2f}/{summary['force_rms_quantiles_N'][2]:.2f} N。误差累积已经有数据证据；这些结果仍不能单独定位某一个网络、损失或训练数据缺陷。</p>
<img src="force_error.png" alt="Increment and cumulative force errors on fixed held-out windows">
<h2>去噪过程中投影的实际尝试</h2><p>受 DPCC 启发，V40 在中间噪声时间从现有可行力序列出发，逐次用真实展开和完整约束沿有限射线保留可接受候选。这是工程上的可行射线检查，不是 DPCC 的精确投影。72 个候选均可接受，只有 4 个降低代价；单组生成耗时约 1.37–7.38 s，不适合 100 ms 预算。这些改进集中于需要重新建立可行种子的早期保存状态。</p>
<p>V46 改为每次 DDIM 去噪后投影输入变化率和六个终端力和式，采用标准化坐标下的加权有界仿射投影。23 个可行教师锚点、五个模型、两个初始化时间，共 920 个候选；原始完整验收 0 个，完整终端修正后 27 个通过，只有 2 个降低代价，均来自同一个理想执行器验证窗口。其余教师锚点未通过修正的一个窗口也保留。仿射残差足够小不能替代非线性动力学、碰撞和终端速度检查。</p>
{table(projection_summary, ['variant','start_t','groups','total','valid','improving','median_ms','p95_ms'])}
<p>V46 使用教师未来力序列建立诊断锚点，衡量投影能否改善生成质量，不能作为独立在线控制方法与基线的公平对比。所有候选均经过同一物理模型和硬约束检查，未更换训练权重，也没有将力指令改成位姿跟踪参考。</p>
<h2>运行速度：不能只看局部改进</h2><p>V38 在十个保存状态上比较串行、2 线程和 4 线程 CasADi stage map，输出误差均为零，但多线程更慢，因此不采用。V41 的有界未缩放 QP 加缩放重试，在八个固定问题上与参考修正一致；六个问题发生首轮不收敛，多次求解的耗时超过参考，因此不采用。全部首轮与最终求解状态分别保留。</p>
<p>V43 初次短时域审计发现 CasADi 将 30 列输入向旧 60 步函数广播；仅修改 H 是错误的。随后增加显式 30 步展开。V43b 原始内核重编译发生 JIT 链接失败；未发现内存耗尽证据，未宣称已确定链接失败原因。V45 改为复用与时域无关的物理内核，三种输入、两种执行器的逐节点一致性检查通过，最大相对误差约 8.4×10⁻¹⁴，生成序列前缀差异为零。</p>
<p>V45 将预测长度从 1.2 s 降为 0.6 s，执行仍为 0.02 s、刷新仍为 0.1 s。在非理想执行器移动障碍任务中，15 s 仿真耗时 34.11 s，刷新中位数/P95 为 75.11/140.15 ms，15/151 次超过 100 ms；但最终位置误差 38.324 mm、姿态误差 3.333°，没有到达。已检查的物理约束通过，零 QP 失败，仍判为任务失败，停止该版本扩展。</p>
<p>V48 保持原来的 60 步、1.2 s 预测，只将反馈展开中的自由力修正分块从 5 步改为 10 步，QP 变量从 72 降为 36。六个冻结状态的矩阵和非线性展开与原问题指定子空间的误差均为零，所有冻结 QP 都收敛。QP 局部计时改善不能直接代表完整控制调用，也不能证明收缩后的决策空间仍可完成任务。以下表格包含实际运行结果。{block_statement}</p>
<p>V48 的实际非理想移动障碍运行最终误差约 38.5 mm，零 QP 失败和零回退仍未到达，故不采用。随后 V50 保留前端和终端的 5 步分块，仅合并中段，使用 [5,10,10,10,10,5,5,5] 共八块、48 个变量。它保留 1.2 s 动力学预测与同一物理验收，并独立审计相对于原 72 变量问题的子空间映射，再运行 Simscape；这些属于不同开发版本，未混合结果。</p>
<p>V50 非理想移动障碍的首次非刷新调用在 0.02 s 耗时约 1.17 s，阶段记录定位为 warm_start，并未发生回退：原热启动仍调用原始零修正展开函数，分块入口却只预热新增展开函数。V54 单独预热实际热启动路径，连续两次输出一致性误差为零，再重复同一开发任务。预热耗时保存在 preparation，未计入控制调用；对应闭环结果及最大执行时间见表。完整 751 点的输入、状态和任务条件差异另存 warmup_consistency.json，避免只凭最终误差判断行为一致。</p>
<p>V54 复跑安全到达：首次执行从 1174.865 ms 降到 10.702 ms，全部 751 点的时间、状态、位置、输入、上一输入和目标差异均为零。预热耗时 1.127 s，明确转移到控制开始之前。执行 P95 为 12.560 ms，最大值 26.043 ms，仍有 2/600 次超过 20 ms。刷新 P95 255.928 ms，151/151 次超过 100 ms，故仍未满足 10 Hz；15 s 同步仿真的墙钟耗时 43.333 s。两次复跑的其他耗时差异不能全部归因于预热。</p>
<p>V50 在 6 s 的一次 QP 达到 400 次迭代上限；原始约束残差接近数值精度，但相对驻点残差约 0.0385，不能称为最优或收敛。原控制器继续进行真实候选验收，不以线性可行为由更改求解器状态。这次控制调用的主要长尾在多次 trial_checks；增加迭代上限或重复求解尚无本轮证据支持。</p>
{table(episodes, ['version','task','mode','fault','horizon_seconds','arrival','constraints_valid','final_position_mm','wall_seconds','refresh_median_ms','refresh_p95_ms','refresh_misses','execution_p95_ms','execution_max_ms','execution_misses','qp_failures','fallbacks'])}
<img src="moving_closed_loop.png" alt="Moving task error and complete controller timings">
<p>所有新实验复用 MATLAB PID 38888；每次任务重置物理状态和控制历史。表中 wall 是同步 15 s 仿真的墙钟耗时，冷编译、模型更新、离线指标与磁盘写入另外记录。不同版本尚未随机交错重复，不能作统计显著性或普遍速度提升结论。执行/规划的最大值与超时次数保存于 CSV，P95 不通过不能称为达到 10 Hz。</p>
<h2>文献依据与适用范围</h2><ul>{refs}</ul>
<h2>结论与后续修改的约束</h2><p>优先解决候选在真实力累加及动力学下的质量，再验证是否提供任务进展。下一步训练设计应将实际力累加误差纳入评价，并检查辅助力序列/展开损失；这是本项目待验证的设计，不能从文献直接声称有效。教师的普通闭环窗口和严格终端窗口应分别审计，不能通过放宽硬约束或删除失败来掩盖差异。保留足够预测长度，并通过受检验的控制参数化和内核复用降低计算量。</p>
<p>本轮候选来自关联窗口与模型，不能把 480 或 920 个候选当作独立任务样本。完整硬约束结论覆盖现有审计采样网格，没有连续时间安全、递归可行或组合稳定性证明。正式 IHSID＋力前馈 LQI＋QP 对比、新方法五种子与 1440 次实验尚未执行。</p>
<h2>复算与证据</h2><p><a href="summary.json">汇总</a> · <a href="teacher_windows.csv">教师窗口</a> · <a href="neural_candidates.csv">480 个原始候选</a> · <a href="projection_groups.csv">投影逐组记录</a> · <a href="projection_summary.csv">投影统计</a> · <a href="microbench.csv">微基准</a> · <a href="development_episodes.csv">完整闭环记录</a> · <a href="controller_phases.csv">阶段耗时</a> · <a href="qp_failures.csv">原生 QP 非收敛记录</a> · <a href="warmup_consistency.json">热启动全时序一致性</a> · <a href="trial_inventory.json">全部审计及失败</a> · <a href="artifact_manifest.json">原始数据和源码哈希</a> · <a href="source_snapshot_v54.zip">源码及工程模型原始字节快照</a></p>
<p>复算命令：<code>chapter5/.venv/Scripts/python.exe chapter5/reduced/literature_probe_report.py</code>。源码守卫：<code>{PRODUCTION_HASH}</code>。五个模型哈希与原始兼容性验收相同；源码快照不包含大型教师数据和权重，使用本工作区清单追溯。重新运行 MATLAB 审计必须使用新的输出目录，禁止覆盖失败记录。</p></html>'''
    (OUT / "index.html").write_text(page, encoding="utf-8")
    print(json.dumps(dict(report=str(OUT / "index.html"), v50_regression=admitted_blocking,
                          episodes=len(episodes), formal_completed=0), ensure_ascii=False))


if __name__ == "__main__":
    main()
