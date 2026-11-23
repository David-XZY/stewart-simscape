"""Read-only training-group residual audit; does not fit or change a model."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def audit(model_file, dataset_file):
    model_raw, data_raw = Path(model_file).read_bytes(), Path(dataset_file).read_bytes()
    net, data = json.loads(model_raw), json.loads(data_raw)
    if net["status"] != "trained" or net["datasetHash"] != hashlib.sha256(data_raw).hexdigest():
        raise ValueError("The trained asset does not match this dataset")
    split, groups = np.asarray(data["split"]), np.asarray(data["groups"])
    training = split == "train"
    x, target, groups = np.asarray(data["x"])[training], np.asarray(data["y"])[training], groups[training]
    h = (x-np.asarray(net["inputMean"]))/np.asarray(net["inputScale"])
    for k, layer in enumerate(net["layers"]):
        h = h @ np.asarray(layer["W"]).T + np.asarray(layer["b"])
        if k+1 < len(net["layers"]):
            h = np.tanh(h)
    scale = np.asarray(net["outputScale"])
    mean = h[:, :6]*scale
    variance = np.exp(np.clip(h[:, 6:], -12, 8))*scale**2*net["varianceCalibration"]
    standardized = (target-mean)/np.sqrt(variance)
    if not np.isfinite(standardized).all():
        raise ValueError("Nonfinite standardized residual")
    segments = []
    summaries = []
    for group in dict.fromkeys(groups):
        ids = np.flatnonzero(groups == group)
        if len(ids) < 2 or not np.all(np.diff(ids) == 1):
            raise ValueError("Each training group must be a contiguous ordered trajectory")
        error = standardized[ids]
        segments.append(error-error.mean(0))
        summaries.append({"group": group, "samples": len(ids), "standardizedMean": error.mean(0).tolist()})
    correlations = []
    for lag in (1, 5, 10, 25, 50):
        pairs = [(error[:-lag], error[lag:]) for error in segments if len(error) > lag]
        if not pairs:
            correlations.append({"rowLag": lag, "pairs": 0, "withinGroupCorrelation": None})
            continue
        left, right = np.concatenate([p[0] for p in pairs]), np.concatenate([p[1] for p in pairs])
        numerator = (left*right).sum(0)
        denominator = np.sqrt((left**2).sum(0)*(right**2).sum(0))
        correlations.append({"rowLag": lag, "pairs": len(left),
                             "withinGroupCorrelation": (numerator/np.maximum(denominator, 1e-30)).tolist()})
    return {"scope": "training groups only; diagnostic, no weight/calibration/model-selection changes",
            "assetHash": hashlib.sha256(model_raw).hexdigest(), "datasetHash": net["datasetHash"],
            "timeAxis": "row lag inside ordered source trajectories; timestamps are absent, so no correlation time is fitted",
            "groups": summaries, "correlations": correlations,
            "standardizedRmse": np.sqrt((standardized**2).mean(0)).tolist(),
            "limitation": "Correlated residuals can include model bias, physical disturbances, and finite-difference label noise; this audit does not separate them."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model"); parser.add_argument("dataset"); parser.add_argument("output")
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(audit(args.model, args.dataset), indent=2, allow_nan=False), encoding="utf-8")
