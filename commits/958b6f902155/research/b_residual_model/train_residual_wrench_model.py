"""Train static wrench mean/log-variance; explicit grouped split, restartable run."""
import argparse
import copy
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
from training_io import load_dataset, write_json, export_layers
from startup_inertia_context import anchors, read, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset")
    parser.add_argument("output")
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--seed", type=int, default=20260905)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dataset-audit", help="Independent raw-observation audit required for startup-inertia data")
    args = parser.parse_args()
    import numpy as np
    import torch
    from torch import nn
    torch.manual_seed(args.seed)
    data = load_dataset(args.dataset)
    if data.get("kind") != "residual-training-dataset":
        raise ValueError("Expected physically fitted residual dataset")
    x = np.asarray(data["x"], dtype=np.float64)
    y = np.asarray(data["y"], dtype=np.float64)
    if x.shape[1] != 12 or y.shape[1] != 6 or not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError("Invalid wrench dataset dimensions or values")
    anchor = anchors(data)
    audit = None
    if anchor is not None:
        if args.dataset_audit is None:
            raise ValueError("Startup-inertia data requires an independent dataset audit")
        audit = read(args.dataset_audit)
        if not (audit.get("passed") is True and audit.get("state") == "complete"
                and audit.get("datasetHash") == data["datasetHash"]
                and audit.get("rows") == len(x) and audit.get("contexts") == len(data["inertiaContexts"])):
            raise ValueError("Dataset audit is incomplete or belongs to other observations")
    elif args.dataset_audit is not None:
        raise ValueError("Startup audit supplied for an unanchored dataset")
    masks = {s: np.asarray(data["split"]) == s for s in ("train", "validation", "test")}
    xm = x[masks["train"]].mean(0)
    xs = np.maximum(x[masks["train"]].std(0), 1e-6)
    ys = np.maximum(y[masks["train"]].std(0), 1.)
    X = torch.tensor((x-xm)/xs, dtype=torch.float32, device=args.device)
    Anchor = None if anchor is None else torch.tensor((anchor-xm)/xs, dtype=torch.float32, device=args.device)
    Y = torch.tensor(y/ys, dtype=torch.float32, device=args.device)
    ids = {s: torch.tensor(np.flatnonzero(m), device=args.device) for s, m in masks.items()}
    model = nn.Sequential(nn.Linear(12, 64), nn.Tanh(), nn.Linear(64, 64), nn.Tanh(), nn.Linear(64, 12)).to(args.device)
    opt = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-5)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    checkpoint = output / "checkpoint.pt"
    start, best, stale, best_state = 0, float("inf"), 0, None
    if args.resume:
        state = torch.load(checkpoint, map_location=args.device, weights_only=True)
        if state["datasetHash"] != data["datasetHash"] or state["seed"] != args.seed:
            raise ValueError("Checkpoint belongs to different data or seed")
        model.load_state_dict(state["model"]); opt.load_state_dict(state["optimizer"])
        start, best, stale, best_state = state["epoch"]+1, state["best"], state["stale"], state["bestState"]
        torch.set_rng_state(state["rng"].cpu())
        if torch.cuda.is_available() and state.get("cudaRng") is not None:
            torch.cuda.set_rng_state_all(state["cudaRng"])
    def loss(index):
        pred = model(X[index]); mu, lv = pred[:, :6], pred[:, 6:].clamp(-12, 8)
        if Anchor is not None:
            mu = mu-model(Anchor[index])[:, :6]
        return .5*(lv+(Y[index]-mu).square()*(-lv).exp()).mean()
    for epoch in range(start, args.epochs):
        model.train()
        order = ids["train"][torch.randperm(len(ids["train"]), device=args.device)]
        for batch in order.split(256):
            opt.zero_grad(); value = loss(batch); value.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.); opt.step()
        model.eval()
        with torch.no_grad():
            val = float(loss(ids["validation"]))
        if val < best:
            best, stale, best_state = val, 0, copy.deepcopy(model.state_dict())
        else:
            stale += 1
        torch.save({"datasetHash": data["datasetHash"], "seed": args.seed, "epoch": epoch,
                    "model": model.state_dict(), "optimizer": opt.state_dict(), "rng": torch.get_rng_state(), "cudaRng": torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None,
                    "best": best, "stale": stale, "bestState": best_state}, checkpoint)
        with (output / "training.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"epoch": epoch, "validationNll": val})+"\n")
        if stale >= 30:
            break
    if best_state is None:
        raise RuntimeError("No trained checkpoint; epochs must permit at least one update")
    model.load_state_dict(best_state); model.eval()
    with torch.no_grad():
        pred = model(X).cpu().numpy()
    mean = pred[:, :6]*ys
    if Anchor is not None:
        with torch.no_grad():
            mean -= model(Anchor).cpu().numpy()[:, :6]*ys
    variance = np.exp(np.clip(pred[:, 6:], -12, 8))*ys**2
    val = masks["validation"]
    calibration = float(np.clip(np.mean((y[val]-mean[val])**2/variance[val]), .01, 100.))
    variance *= calibration
    asset = {"schemaVersion": 1, "kind": "residual-wrench-tanh", "status": "trained",
             "datasetHash": data["datasetHash"], "seed": args.seed, "physical": data["physical"],
             "trainingScope": data.get("scope", "grouped dataset identified by datasetHash"),
             "groupCounts": {s: len(set(np.asarray(data["groups"])[mask])) for s, mask in masks.items()},
             "sampleCounts": {s: int(mask.sum()) for s, mask in masks.items()},
             "referencePhysical": data["referencePhysical"],
             "inputMean": xm.tolist(), "inputScale": xs.tolist(), "outputScale": ys.tolist(),
             "varianceCalibration": calibration, "layers": export_layers(model)}
    if anchor is not None:
        asset.update(meanAnchoring=data["meanAnchoring"], datasetAuditHash=sha(args.dataset_audit),
                     datasetAudit=audit, testEvaluationRole=data["testEvaluationRole"])
    write_json(output / "residual_model.json", asset)
    test = masks["test"]
    write_json(output / "holdout_metrics.json", {
        "rmse": np.sqrt(np.mean((y[test]-mean[test])**2, axis=0)).tolist(),
        "physicalOnlyRmse": np.sqrt(np.mean(y[test]**2, axis=0)).tolist(),
        "scope": asset["trainingScope"],
        "testEvaluationRole": data.get("testEvaluationRole", "frozen holdout evaluation"),
        "coverage95": np.mean(np.abs(y[test]-mean[test]) <= 1.95996398454*np.sqrt(variance[test]), axis=0).tolist(),
        "testGroups": sorted(set(np.asarray(data["groups"])[test]))})


if __name__ == "__main__":
    main()
