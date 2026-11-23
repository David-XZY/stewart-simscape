"""Verify a real CUDA B delivery against its frozen data and independent NumPy values."""
import argparse
from collections import Counter
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'b_residual_model'))
from startup_inertia_context import anchors


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def verify(dataset_file, model_dir, input_manifest, run_status, job_file, tasks_file, collection_file):
    dataset_file, model_dir = Path(dataset_file), Path(model_dir)
    data = read(dataset_file)
    asset_file = model_dir / "residual_model.json"
    asset, manifest, status, job = map(read, (asset_file, input_manifest, run_status, job_file))
    digest = hashlib.sha256(dataset_file.read_bytes()).hexdigest()
    assert data["kind"] == "residual-training-dataset" and asset["status"] == "trained"
    assert digest == asset["datasetHash"].lower() == manifest["datasetSha256"].lower(), "Dataset binding differs"
    assert status["state"] == "complete" and status["exitCode"] == 0, "Training has no successful terminal status"
    args = job["arguments"]
    assert args[args.index("--device") + 1] == "cuda", "Expected the authorized CUDA training job"
    tasks, collection = read(tasks_file), read(collection_file)
    if isinstance(collection, dict):
        assert collection["state"] == "complete" and collection["passed"], "Replay collection is not accepted"
        assert collection["completed"] == collection["expected"]
        collection = collection["originalCollection"]
    expected = {task["taskId"]: task["split"] for task in tasks}
    assert len(expected) == len(tasks) == len(collection), "Incomplete frozen task accounting"
    assert {row["taskId"] for row in collection} == set(expected)
    assert {row["taskId"] for row in collection if row["status"] == "collected"} == set(data["groups"])
    groups, split = np.asarray(data["groups"]), np.asarray(data["split"])
    assert all(expected[group] == part for group, part in zip(groups, split))
    x, y = np.asarray(data["x"], dtype=float), np.asarray(data["y"], dtype=float)
    assert x.shape == (len(groups), 12) and y.shape == (len(groups), 6)
    assert np.isfinite(x).all() and np.isfinite(y).all()
    masks = {part: split == part for part in ("train", "validation", "test")}
    assert all(mask.any() for mask in masks.values())
    expected_means = x[masks["train"]].mean(0)
    expected_scales = np.maximum(x[masks["train"]].std(0), 1e-6)
    expected_outputs = np.maximum(y[masks["train"]].std(0), 1.)
    for recorded, actual in ((asset["inputMean"], expected_means), (asset["inputScale"], expected_scales),
                             (asset["outputScale"], expected_outputs)):
        assert np.allclose(recorded, actual, rtol=1e-10, atol=1e-12), "Normalization is not the frozen train-only fit"
    train_groups = set(groups[masks["train"]])
    assert set(asset["physical"]["trainingGroups"]) == train_groups, "Physical fit has different groups"
    h = (x - asset["inputMean"]) / asset["inputScale"]
    for index, layer in enumerate(asset["layers"]):
        h = h @ np.asarray(layer["W"]).T + layer["b"]
        if index + 1 < len(asset["layers"]):
            h = np.tanh(h)
    assert h.shape == (len(x), 12) and np.isfinite(h).all()
    mean = h[:, :6] * asset["outputScale"]
    anchor = anchors(data)
    if anchor is not None:
        assert asset.get("meanAnchoring") == data["meanAnchoring"]
        assert asset.get("testEvaluationRole") == data["testEvaluationRole"]
        audit = asset["datasetAudit"]
        assert audit["passed"] is True and audit["datasetHash"] == digest and audit["rows"] == len(x)
        assert asset["physical"] == data["physical"]
        a = (anchor-asset["inputMean"])/asset["inputScale"]
        for index, layer in enumerate(asset["layers"]):
            a = a @ np.asarray(layer["W"]).T + layer["b"]
            if index+1 < len(asset["layers"]):
                a = np.tanh(a)
        mean -= a[:, :6]*asset["outputScale"]
    else:
        assert "meanAnchoring" not in asset
    variance = np.exp(np.clip(h[:, 6:], -12, 8)) * np.asarray(asset["outputScale"]) ** 2
    val = masks["validation"]
    calibration = float(np.clip(np.mean((y[val] - mean[val]) ** 2 / variance[val]), .01, 100.))
    assert np.isclose(calibration, asset["varianceCalibration"], rtol=2e-5, atol=1e-7), "Calibration differs from validation data"
    variance *= asset["varianceCalibration"]
    test = masks["test"]
    rmse = np.sqrt(np.mean((y[test] - mean[test]) ** 2, axis=0))
    coverage = np.mean(np.abs(y[test] - mean[test]) <= 1.95996398454 * np.sqrt(variance[test]), axis=0)
    metrics = read(model_dir / "holdout_metrics.json")
    if anchor is not None:
        assert metrics["testEvaluationRole"] == data["testEvaluationRole"]
    assert np.allclose(rmse, metrics["rmse"], rtol=1e-5, atol=1e-4), "Independent test RMSE differs"
    assert np.allclose(coverage, metrics["coverage95"], rtol=0, atol=1 / test.sum()), "Independent test coverage differs"
    assert np.allclose(np.sqrt(np.mean(y[test] ** 2, axis=0)), metrics["physicalOnlyRmse"])
    group_counts = {part: len(set(groups[mask])) for part, mask in masks.items()}
    sample_counts = {part: int(mask.sum()) for part, mask in masks.items()}
    assert group_counts == asset["groupCounts"] and sample_counts == asset["sampleCounts"]
    assert sorted(set(groups[test])) == metrics["testGroups"]
    history = [json.loads(line) for line in (model_dir / "training.jsonl").read_text().splitlines()]
    assert history and all(np.isfinite(row["validationNll"]) for row in history)
    return {"state": "training-complete", "passed": True, "datasetSha256": digest,
            "assetSha256": hashlib.sha256(asset_file.read_bytes()).hexdigest(),
            "collection": dict(Counter(row["status"] for row in collection)),
            "groupCounts": group_counts, "sampleCounts": sample_counts, "epochsExecuted": len(history),
            "selectedEpoch": min(history, key=lambda row: row["validationNll"])["epoch"],
            "deviceArgument": "cuda", "remoteFinished": status["finished"], "holdoutMetrics": metrics,
            "normalizationTrainOnlyPassed": True, "calibrationValidationOnlyPassed": True,
            "maxNumpyTorchRmseDifference": float(np.max(np.abs(rmse - metrics["rmse"]))),
            "scope": "Frozen grouped static-model delivery; closed-loop and full ABFD acceptance remain separate"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("dataset", "model_dir", "input_manifest", "run_status", "job", "tasks", "collection", "output"):
        parser.add_argument(name)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        raise ValueError("Use a fresh verification output")
    report = verify(args.dataset, args.model_dir, args.input_manifest, args.run_status,
                    args.job, args.tasks, args.collection)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(report, allow_nan=False))


if __name__ == "__main__":
    main()
