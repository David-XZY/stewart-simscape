"""Fit a stable residual VAR on training trajectories; select/calibrate on validation.

The static residual network remains unchanged. Test trajectories are evaluated
once after selection. This artifact describes colored uncertainty, not a new
trained mean network or a claim of closed-loop coverage.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

CHI2_6_95 = 12.591587243743977


def stationary_covariance(matrix, noise):
    if np.max(np.abs(np.linalg.eigvals(matrix))) >= 1:
        raise ValueError("Unstable temporal model")
    covariance, power = noise.copy(), matrix.copy()
    for _ in range(40):
        covariance += power @ covariance @ power.T
        power = power @ power
        if np.linalg.norm(power, ord="fro") < 1e-12:
            break
    else:
        raise ValueError("Stationary covariance did not converge")
    covariance = (covariance + covariance.T) / 2
    residual = covariance - matrix @ covariance @ matrix.T - noise
    if np.linalg.norm(residual) > 1e-8 * max(1, np.linalg.norm(covariance)):
        raise ValueError("Stationary covariance does not solve the Lyapunov equation")
    np.linalg.cholesky(covariance)
    return covariance


def paired(segments, order, center):
    history, target = [], []
    for segment in segments:
        values = segment - center
        if len(values) <= order:
            raise ValueError("Trajectory too short for the fixed candidate orders")
        history.append(np.concatenate([values[order-lag-1:-lag-1] for lag in range(order)], axis=1))
        target.append(values[order:])
    return np.concatenate(history), np.concatenate(target)


def innovations(segments, order, center, coefficients, covariance):
    history, target = paired(segments, order, center)
    error = target - history @ coefficients.T
    whitened = np.linalg.solve(np.linalg.cholesky(covariance), error.T).T
    mahalanobis = np.sum(whitened**2, axis=1)
    nll = (6*np.log(2*np.pi) + np.linalg.slogdet(covariance)[1] + mahalanobis).mean()/2
    return error, mahalanobis, float(nll)


def fit(model_file, original_dataset_file, timed_dataset_file, context_tasks_file=None):
    model_raw = Path(model_file).read_bytes()
    original_raw = Path(original_dataset_file).read_bytes()
    timed_raw = Path(timed_dataset_file).read_bytes()
    net, original, data = json.loads(model_raw), json.loads(original_raw), json.loads(timed_raw)
    if net["status"] != "trained" or net["datasetHash"] != hashlib.sha256(original_raw).hexdigest():
        raise ValueError("Static asset does not match the original dataset")
    for field in ("x", "y", "groups", "split"):
        a, b = np.asarray(original[field]), np.asarray(data[field])
        same = np.array_equal(a, b) if field != "y" else (a.shape == b.shape and np.allclose(a, b, rtol=0, atol=1e-10))
        if not same:
            raise ValueError("Timed dataset changed original " + field)
    x, target, time = np.asarray(data["x"]), np.asarray(data["y"]), np.asarray(data["time"])
    groups, split = np.asarray(data["groups"]), np.asarray(data["split"])
    if time.shape != (len(x),) or not np.isfinite(time).all():
        raise ValueError("Invalid original timestamps")
    h = (x-np.asarray(net["inputMean"]))/np.asarray(net["inputScale"])
    for index, layer in enumerate(net["layers"]):
        h = h @ np.asarray(layer["W"]).T + np.asarray(layer["b"])
        if index+1 < len(net["layers"]):
            h = np.tanh(h)
    scale = np.asarray(net["outputScale"])
    mean = h[:, :6]*scale
    conditioning = None
    if "meanAnchoring" in net or "meanAnchoring" in data:
        from startup_inertia_context import anchors, KIND
        anchor = anchors(data)
        if anchor is None or original.get("anchorX") != data.get("anchorX") or net.get("meanAnchoring") != data.get("meanAnchoring"):
            raise ValueError("Anchored temporal data differs from trained mean protocol")
        audit = net.get("datasetAudit", {})
        if not (audit.get("passed") is True and audit.get("datasetHash") == net["datasetHash"]
                and audit.get("rows") == len(x) and audit.get("contexts") == len(data["inertiaContexts"])):
            raise ValueError("Anchored temporal data has no matching independent observation audit")
        if context_tasks_file is None or hashlib.sha256(Path(context_tasks_file).read_bytes()).hexdigest() != audit.get("tasksHash"):
            raise ValueError("Anchored temporal task manifest differs from independent audit")
        if net["physical"] != data["physical"] or original.get("inertiaContexts") != data.get("inertiaContexts"):
            raise ValueError("Startup observations or shared physical model changed")
        a = (anchor-np.asarray(net["inputMean"]))/np.asarray(net["inputScale"])
        for index, layer in enumerate(net["layers"]):
            a = a @ np.asarray(layer["W"]).T + np.asarray(layer["b"])
            if index+1 < len(net["layers"]):
                a = np.tanh(a)
        mean -= a[:, :6]*scale
        conditioning = {"kind": KIND, "contextDatasetHash": hashlib.sha256(timed_raw).hexdigest(),
                        "tasksHash": audit["tasksHash"], "contexts": audit["contexts"],
                        "verificationImplementationHash": audit["verificationImplementationHash"],
                        "maxIndependentOffsetError": audit["maxIndependentError"]["staticOffset"],
                        "maxIndependentInertiaError": audit["maxIndependentError"]["physical"],
                        "maxIndependentLabelError": audit["maxIndependentError"]["label"],
                        "datasetAuditHash": net["datasetAuditHash"],
                        "scope": "Additional independent startup inertia/static observations; anchored residual; historical test diagnostic only"}
    elif "conditioning" in data or "initialBalanceContexts" in data:
        if context_tasks_file is None:
            raise ValueError("Conditional uncertainty requires the frozen task manifest and independent observation verification")
        source = Path(__file__).with_name("static_balance_context.py")
        spec = importlib.util.spec_from_file_location("static_balance_context", source)
        verifier = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(verifier)
        offset, conditioning = verifier.verify(model_file, original_dataset_file, timed_dataset_file, context_tasks_file)
        mean += offset
    elif context_tasks_file is not None:
        raise ValueError("Context manifest provided for an unconditioned dataset")
    deviation = np.sqrt(np.exp(np.clip(h[:, 6:], -12, 8))*scale**2*net["varianceCalibration"])
    standardized = (target-mean)/deviation
    if not np.isfinite(standardized).all():
        raise ValueError("Invalid standardized residual")
    segments = {name: [] for name in ("train", "validation", "test")}
    group_ids = {name: [] for name in segments}
    sample_time = None
    for group in dict.fromkeys(groups):
        ids = np.flatnonzero(groups == group)
        if len(ids) < 10 or not np.all(np.diff(ids) == 1) or len(set(split[ids])) != 1:
            raise ValueError("Each group must be one ordered, split-isolated trajectory")
        difference = np.diff(time[ids])
        if sample_time is None:
            sample_time = float(np.median(difference))
        if sample_time <= 0 or not np.allclose(difference, sample_time, rtol=0, atol=1e-8):
            raise ValueError("VAR requires a common uniform original sample clock")
        segments[split[ids[0]]].append(standardized[ids])
        group_ids[split[ids[0]]].append(str(group))
    if not all(segments.values()):
        raise ValueError("All frozen splits are required")
    center = np.concatenate(segments["train"]).mean(0)
    candidates, accepted = [], []
    for order in (1, 2, 4, 8):
        history, target = paired(segments["train"], order, center)
        gram = history.T @ history
        ridge = 1e-4 * np.trace(gram) / len(gram)
        coefficients = np.linalg.solve(gram+ridge*np.eye(len(gram)), history.T @ target).T
        matrix = np.zeros((6*order, 6*order)); matrix[:6] = coefficients
        if order > 1:
            matrix[6:, :-6] = np.eye(6*(order-1))
        radius = float(np.max(np.abs(np.linalg.eigvals(matrix))))
        row = {"order": order, "spectralRadius": radius, "stable": radius < 1}
        if radius < 1:
            error = target-history @ coefficients.T
            covariance = error.T @ error / len(error)
            np.linalg.cholesky(covariance)
            _, scores, nll = innovations(segments["validation"], order, center, coefficients, covariance)
            row["validationNll"] = nll
            accepted.append((nll, order, matrix, covariance, scores))
        candidates.append(row)
    if not accepted:
        raise ValueError("Every fixed temporal-model candidate was unstable")
    _, order, matrix, covariance, validation_scores = min(accepted, key=lambda item: item[0])
    # Empirical validation quantile; correlated rows are not independent trials.
    calibration = max(1.0, float(np.quantile(validation_scores, .95, method="higher"))/CHI2_6_95)
    covariance *= calibration
    noise = np.zeros_like(matrix); noise[:6, :6] = covariance
    initial = stationary_covariance(matrix, noise)
    evaluation = {}
    for name in ("validation", "test"):
        _, scores, nll = innovations(segments[name], order, center, matrix[:6], covariance)
        unconditioned = np.concatenate(segments[name])-center
        scaled = np.linalg.solve(np.linalg.cholesky(initial[:6, :6]), unconditioned.T).T
        evaluation[name] = {"innovationNll": nll, "conditionalInnovationCoverage95": float(np.mean(scores <= CHI2_6_95)),
                            "stationaryWrenchCoverage95": float(np.mean(np.sum(scaled**2, axis=1) <= CHI2_6_95)),
                            "samples": len(unconditioned)}
    result = {"schemaVersion": 1, "kind": "standardized-residual-var", "status": "calibrated-pilot",
            "staticAssetHash": hashlib.sha256(model_raw).hexdigest(),
            "originalDatasetHash": hashlib.sha256(original_raw).hexdigest(),
            "timedDatasetHash": hashlib.sha256(timed_raw).hexdigest(),
            "implementationHash": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "sampleTime": sample_time, "order": order, "standardizedMean": center.tolist(),
            "transition": matrix.tolist(), "processCovariance": noise.tolist(),
            "initialCovariance": initial.tolist(), "validationScale": calibration,
            "groups": group_ids, "candidates": candidates, "evaluation": evaluation,
            "stateDefinition": "[centered standardized wrench error at t; t-1; ...]; positive error subtracts from forward dynamics",
            "protocol": "fit/center/ridge on train only; order selection and innovation scaling on validation only; test evaluated after selection",
            "scope": "pilot temporal uncertainty calibration; static network unchanged; not closed-loop or full-data acceptance",
            "accelerationSource": data.get("accelerationSource", "See original logged sessions"),
            "limitations": "Stationary VAR and validation calibration do not establish broad or closed-loop coverage; model error and measurement/label noise are not separated."}
    if conditioning is not None:
        result["conditioning"] = conditioning
        result["scope"] = "pilot temporal calibration after independently measured initial static balance; weights unchanged; not closed-loop or full-data acceptance"
        result["limitations"] += " Requires an additional independent static-balance observation for every task, including validation/test; observation error is not separately identified."
        if "meanAnchoring" in net:
            result["testEvaluationRole"] = net["testEvaluationRole"]
            result["scope"] = "pilot uncertainty for startup-inertia/static-conditioned anchored B; no closed-loop acceptance"
            result["limitations"] += " Inertia also requires a pre-task force/acceleration response measurement. Previously exposed test is diagnostic only."
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model"); parser.add_argument("original_dataset")
    parser.add_argument("timed_dataset"); parser.add_argument("output")
    parser.add_argument("--context-tasks", help="Frozen task manifest, required only for independent static-balance conditioning")
    args = parser.parse_args()
    result = fit(args.model, args.original_dataset, args.timed_dataset, args.context_tasks)
    output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise ValueError("Use a fresh output file; calibration artifacts are immutable")
    output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"order": result["order"], "scale": result["validationScale"], "evaluation": result["evaluation"]}))
