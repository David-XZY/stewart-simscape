"""Check exported F sampling against independent checkpoint/MATLAB references."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "f_trajectory_proposal"))
from sample_trajectory_candidates import sample


def verify(root):
    root = Path(root)
    read = lambda file: json.loads(file.read_text(encoding="utf-8-sig"))
    sha = lambda file: hashlib.sha256(file.read_bytes()).hexdigest()
    reference = read(root / "import_checks/torch_reference.json")
    matlab = read(root / "import_checks/matlab_import.json")
    manifest = read(root / "task_manifest.json")
    data = read(root / "proposal_dataset.json")
    attempts = read(root / "proposal_attempts.json")
    dataset_hash = sha(root / "proposal_dataset.json")
    assert reference["datasetHash"] == matlab["datasetHash"] == dataset_hash
    assert matlab["passed"]
    expected = {(task["taskId"], start) for task in manifest["tasks"]
                for start in range(1, manifest["candidateCount"] + 1)}
    actual = [(row["taskId"], row["start"]) for row in attempts]
    assert len(actual) == len(set(actual)) and set(actual) == expected
    assert all(type(row["passed"]) is bool and row["reason"] for row in attempts)
    accepted = {(row["taskId"], row["start"]) for row in attempts if row["passed"]}
    rows = list(zip(data["groups"], data["startIndex"]))
    assert len(rows) == len(set(rows)) and set(rows) == accepted
    frozen_split = {task["taskId"]: task["split"] for task in manifest["tasks"]}
    assert all(frozen_split[group] == split for group, split in zip(data["groups"], data["split"]))
    errors = {}
    previous_nearest_errors = []
    for mode in ("nearest", "mlp"):
        file = root / "models" / mode / "proposal_model.json"
        asset = read(file)
        assert sha(file) == matlab[f"{mode}AssetHash"] and asset["datasetHash"] == dataset_hash
        differences = []
        for index, context in enumerate(reference["contexts"]):
            task = {"context": context, "scene": {"q0": context[:6], "qWaypoint": context[6:12]}}
            cp = np.asarray(sample(asset, task, 8, 20260905))
            assert cp.shape == (8, 6, 10)
            assert np.all(cp[:, :, :3] == np.asarray(context[:6]).reshape(1, 6, 1))
            assert np.all(cp[:, :, 7:] == np.asarray(context[6:12]).reshape(1, 6, 1))
            feature = np.asarray([value[:, 3:7].reshape(-1, order="F") for value in cp])
            target = np.asarray(reference[f"{mode}Features"][index])
            differences.append(float(np.max(np.abs(feature - target))))
            if mode == "nearest":
                # Reproduce the previous production algorithm without changing
                # its source or weights; keep the discrepancy as evidence.
                distance = np.sum(((np.asarray(asset["contexts"])-context)/asset["inputScale"])**2, axis=1)
                old = np.asarray(asset["features"])[np.argsort(distance)[:8]]
                previous_nearest_errors.append(float(np.max(np.abs(old-target))))
        errors[mode] = max(differences)
        assert errors[mode] < 1e-12
    return {"passed": True, "datasetHash": dataset_hash, "tasks": len(manifest["tasks"]),
            "attempts": len(attempts), "accepted": len(accepted),
            "zeroSuccessTaskIds": sorted(set(frozen_split)-set(data["groups"])),
            "checkedContexts": len(reference["contexts"]), "candidatesPerContextPerMode": 8,
            "maximumFeatureError": errors,
            "previousUnstableNearest": {"mismatchedContexts": sum(e > 1e-12 for e in previous_nearest_errors),
                                        "maximumFeatureError": max(previous_nearest_errors)},
            "scope": "All frozen attempts accounted for; Python sampling agrees with independently checked checkpoint/MATLAB references; no optimizer acceptance claim"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    report = verify(args.root)
    text = json.dumps(report, indent=2, allow_nan=False)
    (args.root / "import_checks/python_sampling.json").write_text(text, encoding="utf-8")
    print(text)
