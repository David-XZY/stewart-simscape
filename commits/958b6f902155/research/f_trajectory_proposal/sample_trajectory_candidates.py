"""Sample exported proposal weights without PyTorch; results are only NLP initial guesses."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
from training_io import write_json


def sample(asset, task, count, seed):
    import numpy as np
    if asset.get("status") != "trained" or asset.get("kind") != "trajectory-proposal":
        raise ValueError("A trained trajectory proposal asset is required")
    context = np.asarray(task["context"]).reshape(-1)
    scale = np.asarray(asset["inputScale"]).reshape(-1)
    mean = np.asarray(asset["inputMean"]).reshape(-1)
    z = (context-mean)/scale
    rng = np.random.default_rng(seed)
    def mlp(h):
        for index, layer in enumerate(asset["layers"]):
            h = np.asarray(layer["W"]) @ h + np.asarray(layer["b"]).reshape(-1)
            if index < len(asset["layers"])-1:
                h = np.tanh(h)
        return h
    if asset["mode"] == "nearest":
        distance = np.sum(((np.asarray(asset["contexts"])-context)/scale)**2, axis=1)
        # Multi-start rows share task contexts; retain library order for ties,
        # matching MATLAB sort and keeping the top-eight candidate set stable.
        order = np.argsort(distance, kind="stable")
        features = [np.asarray(asset["features"])[order[k % len(order)]] for k in range(count)]
    else:
        features = []
        for _ in range(count):
            if asset["mode"] == "mlp":
                value = mlp(z)
            elif asset["mode"] == "diffusion":
                beta = np.asarray(asset["beta"]).reshape(-1)
                abar = np.cumprod(1-beta); value = rng.standard_normal(24)
                for s in range(len(beta)-1, -1, -1):
                    phase = s/(len(beta)-1)*np.pi
                    noise = mlp(np.concatenate([z, value, [np.sin(phase), np.cos(phase)]]))
                    value = (value-beta[s]/np.sqrt(1-abar[s])*noise)/np.sqrt(1-beta[s])
                    if s:
                        value += np.sqrt(beta[s]*(1-abar[s-1])/(1-abar[s]))*rng.standard_normal(24)
            else:
                raise ValueError("Unknown proposal model")
            features.append(value*np.asarray(asset["outputScale"])+np.asarray(asset["outputMean"]))
    candidates = []
    for feature in features:
        cp = np.zeros((6, 10))
        cp[:, :3] = np.asarray(task["scene"]["q0"]).reshape(6, 1)
        cp[:, 7:] = np.asarray(task["scene"]["qWaypoint"]).reshape(6, 1)
        cp[:, 3:7] = np.asarray(feature).reshape(6, 4, order="F")
        if not np.isfinite(cp).all():
            raise ValueError("Nonfinite proposal")
        candidates.append(cp.tolist())
    return candidates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("asset"); parser.add_argument("task"); parser.add_argument("output")
    parser.add_argument("--count", type=int, default=8); parser.add_argument("--seed", type=int, default=20260905)
    args = parser.parse_args()
    if args.count < 1:
        parser.error("count must be positive")
    asset = json.loads(Path(args.asset).read_text(encoding="utf-8"))
    task = json.loads(Path(args.task).read_text(encoding="utf-8"))
    write_json(args.output, {"schemaVersion": 1, "status": "unrefined-candidates",
                            "controlPoints": sample(asset, task, args.count, args.seed)})


if __name__ == "__main__":
    main()
