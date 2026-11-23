"""Conditional MLP/DDPM or train-only nearest-neighbor trajectory library."""
import argparse
import copy
import json
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
from training_io import load_dataset, write_json, export_layers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset"); parser.add_argument("output")
    parser.add_argument("--mode", choices=["nearest", "mlp", "diffusion"], default="nearest")
    parser.add_argument("--audit")
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--seed", type=int, default=20260905)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    import numpy as np
    data = load_dataset(args.dataset)
    if data.get("kind") != "trajectory-proposal-dataset":
        raise ValueError("Expected trajectory proposal dataset")
    if args.mode == "diffusion":
        if not args.audit:
            raise ValueError("Diffusion requires an F0 audit")
        audit = json.loads(Path(args.audit).read_text(encoding="utf-8"))
        if not audit.get("diffusionApproved") or audit.get("datasetHash") != data["datasetHash"]:
            raise ValueError("Multimodality audit did not approve this dataset")
    x, y = np.asarray(data["x"], dtype=float), np.asarray(data["y"], dtype=float)
    split = np.asarray(data["split"])
    if y.shape[1] != 24 or not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError("Invalid spline feature dataset")
    xm, xs = x[split == "train"].mean(0), np.maximum(x[split == "train"].std(0), 1e-6)
    ym, ys = y[split == "train"].mean(0), np.maximum(y[split == "train"].std(0), 1e-6)
    asset = {"schemaVersion": 1, "kind": "trajectory-proposal", "mode": args.mode, "status": "trained",
             "datasetHash": data["datasetHash"], "inputMean": xm.tolist(), "inputScale": xs.tolist(),
             "outputMean": ym.tolist(), "outputScale": ys.tolist(), "seed": args.seed}
    asset["trainingScope"] = data.get("trainingScope", "Dataset scope is defined by its source manifest")
    asset["groupCounts"] = {s: len({g for g, v in zip(data["groups"], data["split"]) if v == s})
                            for s in ("train", "validation", "test")}
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    if args.mode == "nearest":
        asset.update(contexts=x[split == "train"].tolist(), features=y[split == "train"].tolist())
        write_json(output / "proposal_model.json", asset)
        return
    import torch
    from torch import nn
    torch.manual_seed(args.seed)
    X = torch.tensor((x-xm)/xs, dtype=torch.float32, device=args.device)
    Y = torch.tensor((y-ym)/ys, dtype=torch.float32, device=args.device)
    ids = {s: torch.tensor(np.flatnonzero(split == s), device=args.device) for s in ("train", "validation")}
    diffusion = args.mode == "diffusion"; steps = 100
    beta = torch.linspace(1e-4, .02, steps, device=args.device)
    abar = torch.cumprod(1-beta, 0)
    width = x.shape[1]+26 if diffusion else x.shape[1]
    model = nn.Sequential(nn.Linear(width, 128), nn.Tanh(), nn.Linear(128, 128), nn.Tanh(), nn.Linear(128, 24)).to(args.device)
    opt = torch.optim.AdamW(model.parameters(), lr=1e-3)
    best, best_state, stale, start = math.inf, None, 0, 0
    checkpoint = output / "checkpoint.pt"
    if args.resume:
        state = torch.load(checkpoint, map_location=args.device, weights_only=True)
        if state["datasetHash"] != data["datasetHash"] or state["mode"] != args.mode or state["seed"] != args.seed:
            raise ValueError("Incompatible proposal checkpoint")
        model.load_state_dict(state["model"]); opt.load_state_dict(state["optimizer"])
        best, best_state, stale, start = state["best"], state["bestState"], state["stale"], state["epoch"]+1
        torch.set_rng_state(state["rng"].cpu())
        if torch.cuda.is_available() and state.get("cudaRng") is not None:
            torch.cuda.set_rng_state_all(state["cudaRng"])
    def loss(index, fixed=False):
        if not diffusion:
            return (model(X[index])-Y[index]).square().mean()
        generator = torch.Generator(device=args.device).manual_seed(args.seed+19) if fixed else None
        t = torch.randint(steps, (len(index),), device=args.device, generator=generator)
        noise = torch.randn(Y[index].shape, device=args.device, generator=generator)
        at = abar[t, None]
        z = at.sqrt()*Y[index]+(1-at).sqrt()*noise
        phase = t[:, None]/(steps-1)*math.pi
        inputs = torch.cat([X[index], z, phase.sin(), phase.cos()], 1)
        return (model(inputs)-noise).square().mean()
    for epoch in range(start, args.epochs):
        model.train(); order = ids["train"][torch.randperm(len(ids["train"]), device=args.device)]
        for batch in order.split(128):
            opt.zero_grad(); value = loss(batch); value.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.); opt.step()
        model.eval()
        with torch.no_grad():
            val = float(loss(ids["validation"], True))
        if val < best:
            best, best_state, stale = val, copy.deepcopy(model.state_dict()), 0
        else:
            stale += 1
        torch.save({"datasetHash": data["datasetHash"], "mode": args.mode, "seed": args.seed,
                    "model": model.state_dict(), "optimizer": opt.state_dict(), "epoch": epoch,
                    "best": best, "bestState": best_state, "stale": stale, "rng": torch.get_rng_state(), "cudaRng": torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None}, checkpoint)
        with (output / "training.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"epoch": epoch, "validationLoss": val})+"\n")
        if stale >= 30:
            break
    if best_state is None:
        raise RuntimeError("No trained proposal checkpoint")
    model.load_state_dict(best_state)
    asset["layers"] = export_layers(model)
    if diffusion:
        asset["beta"] = beta.cpu().tolist()
    write_json(output / "proposal_model.json", asset)


if __name__ == "__main__":
    main()
