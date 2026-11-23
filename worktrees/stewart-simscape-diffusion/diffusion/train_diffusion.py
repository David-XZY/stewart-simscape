"""Train/export the actual full-horizon conditional DDPM; no synthetic fallback."""
import argparse
import copy
import hashlib
import json
import time
from pathlib import Path


def write_json(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, allow_nan=False), encoding='utf-8')
    tmp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--device', default='cuda')
    parser.add_argument('--epochs', type=int, default=300)
    parser.add_argument('--seed', type=int, default=20260915)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    started = time.time()
    import numpy as np
    import torch
    from torch import nn
    data = json.loads(args.dataset.read_text(encoding='utf-8-sig'))
    assert data['kind'] == 'stewart-full-diffusion-dataset'
    x, y = np.asarray(data['x'], dtype=float), np.asarray(data['y'], dtype=float)
    split, groups = np.asarray(data['split']), np.asarray(data['groups'])
    assert y.ndim == 2 and y.shape[1] == 60 and x.shape[0] == y.shape[0]
    assert np.isfinite(x).all() and np.isfinite(y).all()
    group_sets = {s: set(groups[split == s]) for s in ('train', 'validation', 'test')}
    assert group_sets['train'] and group_sets['validation']
    assert not any(group_sets[a] & group_sets[b] for a, b in [('train', 'validation'), ('train', 'test'), ('validation', 'test')])
    xm, xs = x[split == 'train'].mean(0), np.maximum(x[split == 'train'].std(0), 1e-6)
    ym, ys = y[split == 'train'].mean(0), np.maximum(y[split == 'train'].std(0), 1e-5)
    torch.manual_seed(args.seed)
    model = nn.Sequential(nn.Linear(x.shape[1]+62, 256), nn.Tanh(), nn.Linear(256, 256), nn.Tanh(),
                          nn.Linear(256, 256), nn.Tanh(), nn.Linear(256, 60)).to(args.device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-3)
    X = torch.tensor((x-xm)/xs, dtype=torch.float32, device=args.device)
    Y = torch.tensor((y-ym)/ys, dtype=torch.float32, device=args.device)
    ids = {s: torch.tensor(np.flatnonzero(split == s), device=args.device) for s in ('train', 'validation')}
    beta = torch.linspace(1e-4, .02, 100, device=args.device)
    abar = torch.cumprod(1-beta, 0)
    args.output.mkdir(parents=True, exist_ok=True)
    fingerprint = hashlib.sha256(args.dataset.read_bytes()).hexdigest()
    checkpoint = args.output/'checkpoint.pt'
    best, state, stale, start = float('inf'), None, 0, 0
    if args.resume:
        old = torch.load(checkpoint, map_location=args.device, weights_only=True)
        assert old['datasetHash'] == fingerprint and old['seed'] == args.seed
        model.load_state_dict(old['model']); optimizer.load_state_dict(old['optimizer'])
        best, state, stale, start = old['best'], old['bestState'], old['stale'], old['epoch']+1
        torch.set_rng_state(old['rng'].cpu())
        if args.device.startswith('cuda'): torch.cuda.set_rng_state_all(old['cudaRng'])

    def loss(index, fixed=False):
        generator = torch.Generator(device=args.device).manual_seed(args.seed+11) if fixed else None
        t = torch.randint(100, (len(index),), device=args.device, generator=generator)
        noise = torch.randn((len(index), 60), device=args.device, generator=generator)
        z = abar[t, None].sqrt()*Y[index]+(1-abar[t, None]).sqrt()*noise
        phase = t[:, None]/99*torch.pi
        estimate = model(torch.cat([X[index], z, phase.sin(), phase.cos()], 1))
        return (estimate-noise).square().mean()

    epoch = start-1
    # A completed early-stop checkpoint is re-exported unchanged on resume.
    stop = args.epochs if stale < 30 else start
    for epoch in range(start, stop):
        model.train()
        order = ids['train'][torch.randperm(len(ids['train']), device=args.device)]
        for batch in order.split(128):
            optimizer.zero_grad(); value = loss(batch); value.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 10); optimizer.step()
        model.eval()
        with torch.no_grad(): validation = float(loss(ids['validation'], True))
        if validation < best: best, state, stale = validation, copy.deepcopy(model.state_dict()), 0
        else: stale += 1
        torch.save({'datasetHash': fingerprint, 'seed': args.seed, 'epoch': epoch, 'model': model.state_dict(),
                    'optimizer': optimizer.state_dict(), 'best': best, 'bestState': state, 'stale': stale,
                    'rng': torch.get_rng_state(), 'cudaRng': torch.cuda.get_rng_state_all() if args.device.startswith('cuda') else []}, checkpoint)
        with (args.output/'training.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps({'epoch': epoch, 'validationNoiseMSE': validation})+'\n')
        if stale >= 30: break
    assert state is not None
    model.load_state_dict(state); model.eval()
    layers = [{'W': layer.weight.detach().cpu().tolist(), 'b': layer.bias.detach().cpu().tolist()}
              for layer in model if isinstance(layer, nn.Linear)]
    asset = {'schemaVersion': 1, 'kind': 'stewart-full-diffusion', 'status': 'trained', 'datasetHash': fingerprint,
             'manifestHash': data['manifestHash'], 'seed': args.seed, 'inputMean': xm.tolist(), 'inputScale': xs.tolist(),
             'outputMean': ym.tolist(), 'outputScale': ys.tolist(), 'beta': beta.cpu().tolist(), 'layers': layers,
             'groupCounts': {s: len(g) for s, g in group_sets.items()}, 'bestValidationNoiseMSE': best,
             'device': args.device, 'torchVersion': torch.__version__, 'epochsCompleted': epoch+1}
    asset['trainingWallSecondsThisInvocation'] = time.time()-started
    asset['trainingConfig'] = {'hiddenWidths': [256, 256, 256], 'activation': 'tanh', 'diffusionSteps': 100,
                               'optimizer': 'AdamW', 'learningRate': 1e-3, 'batchSize': 128,
                               'maxEpochs': args.epochs, 'earlyStoppingPatience': 30}
    write_json(args.output/'model.json', asset)
    # Cross-language reference on deterministic nonzero network inputs.
    probe = torch.linspace(-.5, .5, x.shape[1]+62, device=args.device).reshape(1, -1)
    with torch.no_grad(): expected = model(probe).cpu().tolist()
    write_json(args.output/'network_probe.json', {'input': probe.cpu().tolist(), 'output': expected})
    print(json.dumps({'status': 'trained', 'rows': len(y), 'trainGroups': len(group_sets['train']), 'bestValidationNoiseMSE': best}))


if __name__ == '__main__':
    main()
