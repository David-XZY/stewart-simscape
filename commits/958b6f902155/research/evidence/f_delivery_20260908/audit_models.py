import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import torch

root = Path(sys.argv[1])
output = Path(sys.argv[2])
output.mkdir(parents=True, exist_ok=True)
read = lambda path: json.loads(path.read_text(encoding="utf-8-sig"))
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
data = read(root / "proposal_dataset.json")
mlp = read(root / "models/mlp/proposal_model.json")
nearest = read(root / "models/nearest/proposal_model.json")
audit = read(root / "multimodality_audit.json")
dataset_hash = sha(root / "proposal_dataset.json")
assert dataset_hash == "a74569715048577d1ffa7f8df7b0de0a5fadad6006373cbcd68d1611b908e524"
assert mlp["datasetHash"] == nearest["datasetHash"] == audit["datasetHash"] == dataset_hash
x, y = np.asarray(data["x"], dtype=float), np.asarray(data["y"], dtype=float)
split, groups = np.asarray(data["split"]), np.asarray(data["groups"])
assert x.shape == (364, 72) and y.shape == (364, 24)
for group in set(groups):
    assert len(set(split[groups == group])) == 1
train = split == "train"
expected = {"inputMean": x[train].mean(0), "inputScale": np.maximum(x[train].std(0), 1e-6),
            "outputMean": y[train].mean(0), "outputScale": np.maximum(y[train].std(0), 1e-6)}
normalization_error = 0.0
for model in (mlp, nearest):
    for field, value in expected.items():
        error = float(np.max(np.abs(np.asarray(model[field])-value)))
        normalization_error = max(normalization_error, error)
assert normalization_error < 1e-12
assert np.array_equal(nearest["contexts"], x[train]) and np.array_equal(nearest["features"], y[train])
checkpoint = torch.load(root / "models/mlp/checkpoint.pt", map_location="cpu", weights_only=True)
assert checkpoint["datasetHash"] == dataset_hash and checkpoint["mode"] == "mlp"
weight_error = 0.0
for index, layer in enumerate(mlp["layers"]):
    for suffix, field in (("weight", "W"), ("bias", "b")):
        actual = np.asarray(layer[field]).reshape(checkpoint["bestState"][f"{2*index}.{suffix}"].shape)
        weight_error = max(weight_error, float(np.max(np.abs(actual-checkpoint["bestState"][f"{2*index}.{suffix}"].numpy()))))
assert weight_error == 0
history = [json.loads(line) for line in (root / "models/mlp/training.jsonl").read_text().splitlines()]
best = min(history, key=lambda row: row["validationLoss"])
assert abs(best["validationLoss"]-checkpoint["best"]) < 1e-12
assert checkpoint["epoch"] == history[-1]["epoch"]
indices = np.linspace(0, len(x)-1, 32, dtype=int)
inputs = x[indices]
value = torch.as_tensor((inputs-expected["inputMean"])/expected["inputScale"], dtype=torch.float64)
with torch.no_grad():
    for index in range(3):
        state = checkpoint["bestState"]
        value = value @ state[f"{2*index}.weight"].double().T + state[f"{2*index}.bias"].double()
        if index < 2:
            value = torch.tanh(value)
features = value.numpy()*expected["outputScale"]+expected["outputMean"]
nearest_features = []
nearest_indices = []
for context in inputs:
    distance = np.sum(((x[train]-context)/expected["inputScale"])**2, axis=1)
    order = np.argsort(distance, kind="stable")[:8]
    nearest_indices.append(order.tolist())
    nearest_features.append(y[train][order].tolist())
reference = {"datasetHash": dataset_hash, "taskIds": groups[indices].tolist(), "rowIndices": indices.tolist(),
             "contexts": inputs.tolist(), "mlpFeatures": features.tolist(), "nearestFeatures": nearest_features,
             "nearestRowIndices": nearest_indices,
             "scope": "PyTorch float64 best checkpoint outputs and independent train-only stable nearest rows; technical import checks, no new training"}
report = {"passed": True, "datasetHash": dataset_hash, "normalizationMaximumError": normalization_error,
          "checkpointWeightMaximumError": weight_error, "epochs": len(history), "selectedEpoch": best["epoch"],
          "validationLoss": best["validationLoss"], "referenceRows": len(indices),
          "rowCounts": {name: int(np.sum(split == name)) for name in ("train", "validation", "test")},
          "groupCounts": {name: len(set(groups[split == name])) for name in ("train", "validation", "test")},
          "mlpAssetHash": sha(root / "models/mlp/proposal_model.json"),
          "nearestAssetHash": sha(root / "models/nearest/proposal_model.json"),
          "diffusionApproved": audit["diffusionApproved"],
          "scope": "Independent normalization, grouping, train-only nearest library, checkpoint export and validation selection audit; no trajectory-refinement acceptance"}
for name, value in (("model_audit.json", report), ("torch_reference.json", reference)):
    (output / name).write_text(json.dumps(value, ensure_ascii=True, allow_nan=False), encoding="utf-8")
print(json.dumps(report, allow_nan=False))
