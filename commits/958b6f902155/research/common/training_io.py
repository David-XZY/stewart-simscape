"""Dependency-free dataset contracts used before any training runtime is imported."""
import hashlib
import json
from pathlib import Path


def load_dataset(path):
    raw = Path(path).read_bytes()
    data = json.loads(raw)
    if data.get("schemaVersion") != 1:
        raise ValueError("Unsupported dataset schema")
    n = len(data["x"])
    if not n or any(len(data[k]) != n for k in ("y", "groups", "split")):
        raise ValueError("Dataset columns have different lengths")
    memberships = {}
    for group, split in zip(data["groups"], data["split"]):
        if split not in ("train", "validation", "test"):
            raise ValueError(f"Invalid split {split}")
        if group in memberships and memberships[group] != split:
            raise ValueError(f"Group leakage: {group}")
        memberships[group] = split
    if set(data["split"]) != {"train", "validation", "test"}:
        raise ValueError("Frozen train, validation and test groups are required")
    data["datasetHash"] = hashlib.sha256(raw).hexdigest()
    return data


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    temp.replace(path)


def export_layers(model):
    return [{"W": layer.weight.detach().cpu().tolist(),
             "b": layer.bias.detach().cpu().tolist()}
            for layer in model if hasattr(layer, "weight")]
