"""Task-separated control-increment DDPM. No spline checkpoint is loaded."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
import shutil
import time
import uuid
from pathlib import Path

import h5py
import numpy as np
import scipy.io
import torch
from torch import nn

ROOT = Path(__file__).resolve().parent


class Denoiser(nn.Module):
    def __init__(self, action_dim: int, context_dim: int, hidden: list[int]):
        super().__init__()
        dims = [action_dim + context_dim + 32, *hidden, action_dim]
        self.layers = nn.ModuleList(nn.Linear(a, b) for a, b in zip(dims, dims[1:]))
        # An identity noise skip avoids forcing a 256-wide bottleneck to
        # reproduce 360 independent noise coordinates at high diffusion time.
        nn.init.zeros_(self.layers[-1].weight)
        nn.init.zeros_(self.layers[-1].bias)

    def forward(self, action, context, timestep):
        frequency = torch.exp(-math.log(10000) * torch.arange(16, device=action.device) / 15)
        phase = timestep[:, None] * frequency[None, :]
        value = torch.cat([action, context, phase.sin(), phase.cos()], dim=-1)
        for layer in self.layers[:-1]:
            value = torch.tanh(layer(value))
        return action + self.layers[-1](value)


def read_windows(split):
    with h5py.File(ROOT / 'artifacts' / 'teacher' / f'{split}_windows.h5') as f:
        # MATLAB HDF5 dimension ordering is reversed in h5py.
        context = np.asarray(f['context'], dtype=np.float32)
        action = np.asarray(f['delta'], dtype=np.float32)
        version = f.attrs['version']
    assert action.ndim == 2 and action.shape[1] == 360
    assert context.shape == (action.shape[0], 219)
    assert np.isfinite(action).all() and np.isfinite(context).all()
    return action, context, str(version)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_torch_save(value, path):
    temporary = path.with_suffix(path.suffix + '.pending')
    torch.save(value, temporary)
    os.replace(temporary, path)


def reconcile_training_log(path, completed_epochs):
    """Preserve an interrupted log before aligning it to the durable state.

    A crash between the CSV flush and checkpoint replacement can leave an
    extra epoch in the log. Resume from the checkpoint's RNG/optimizer state
    and retain that interrupted log as evidence, rather than duplicate epochs.
    """
    if not path.exists():
        assert completed_epochs == 0, 'A resume checkpoint requires its training log.'
        return None
    with path.open(newline='', encoding='utf-8') as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames
        rows = list(reader)
    prefix = [row for row in rows if int(row['epoch']) <= completed_epochs]
    assert [int(row['epoch']) for row in prefix] == list(range(1, completed_epochs + 1)), 'Durable training epochs are missing or duplicated.'
    if len(prefix) == len(rows):
        return None
    archive = path.with_name(f'{path.stem}_interrupted_{uuid.uuid4().hex}.csv')
    temporary = path.with_suffix('.csv.pending')
    with temporary.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader();writer.writerows(prefix)
    shutil.copy2(path, archive)
    os.replace(temporary, path)
    return archive.name


def train(seed: int, config: dict):
    out = ROOT / 'artifacts' / 'models'
    out.mkdir(parents=True, exist_ok=True)
    manifest_path = ROOT / 'artifacts' / 'teacher' / 'dataset_manifest.json'
    manifest = json.loads(manifest_path.read_text('utf-8'))
    assert manifest['version'] == config['version']
    assert len(manifest['raw_runs']) == 2 * (config['train_tasks'] + config['validation_tasks'])
    for split in ['train', 'validation']:
        assert sha256(ROOT / 'artifacts' / 'teacher' / f'{split}_windows.h5') == manifest[f'{split}_sha256']
    identity = {'dataset_manifest_sha256': sha256(manifest_path), 'trainer_sha256': sha256(Path(__file__)),
                'configuration_sha256': sha256(ROOT / 'config.json')}
    metadata_path = out / f'seed_{seed}.json'
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text('utf-8'))
        assert metadata['version'] == config['version']
        assert metadata['completed'] and sha256(out / f'seed_{seed}.mat') == metadata['matlab_sha256']
        assert sha256(out / f'seed_{seed}.pt') == metadata['checkpoint_sha256']
        assert all(metadata[key] == value for key, value in identity.items())
        return
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(min(4, os.cpu_count() or 1))
    torch.use_deterministic_algorithms(True)
    train_a, train_c, _ = read_windows('train')
    val_a, val_c, _ = read_windows('validation')
    a_mean, c_mean = train_a.mean(0), train_c.mean(0)
    a_std = np.maximum(train_a.std(0), 1e-3)
    c_std = np.maximum(train_c.std(0), 1e-3)
    ta = torch.from_numpy((train_a - a_mean) / a_std)
    tc = torch.from_numpy((train_c - c_mean) / c_std)
    va = torch.from_numpy((val_a - a_mean) / a_std)
    vc = torch.from_numpy((val_c - c_mean) / c_std)
    steps = config['diffusion_steps']
    grid = torch.arange(steps + 1, dtype=torch.float64)
    cumulative = torch.cos((grid / steps + .008) / 1.008 * math.pi / 2).square()
    cumulative = cumulative / cumulative[0]
    beta = (1 - cumulative[1:] / cumulative[:-1]).clamp(max=.999)
    alpha_bar = torch.cumprod(1 - beta, dim=0).float()
    model = Denoiser(ta.shape[1], tc.shape[1], config['hidden'])
    optimizer = torch.optim.AdamW(model.parameters(), lr=config['learning_rate'])
    fixed = torch.Generator().manual_seed(9991)
    val_t = torch.randint(0, steps, (len(va),), generator=fixed)
    val_noise = torch.randn(va.shape, generator=fixed)
    best = float('inf')
    checkpoint = out / f'seed_{seed}.pt'
    log = out / f'seed_{seed}_training.csv'
    resume = out / f'seed_{seed}_resume.pt'
    first_epoch = 0
    if resume.exists():
        last = torch.load(resume, weights_only=False)
        assert all(last['identity'][key] == value for key, value in identity.items())
        model.load_state_dict(last['model']);optimizer.load_state_dict(last['optimizer'])
        first_epoch = last['next_epoch'];best = last['best']
        torch.set_rng_state(last['torch_rng']);np.random.set_state(last['numpy_rng']);random.setstate(last['python_rng'])
    attempts = out / f'seed_{seed}_attempts.jsonl'
    interrupted_log = reconcile_training_log(log, first_epoch)
    with attempts.open('a', encoding='utf-8') as handle:
        handle.write(json.dumps({'status': 'started', 'utc_seconds': time.time(), 'first_epoch': first_epoch + 1,
                                'interrupted_log': interrupted_log, **identity}) + '\n')
    with log.open('a' if log.exists() else 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=['epoch', 'training_loss', 'validation_loss', 'selected'])
        if handle.tell() == 0:
            writer.writeheader()
        for epoch in range(first_epoch, config['max_epochs']):
            model.train()
            permutation = torch.randperm(len(ta))
            total = 0.
            for indices in permutation.split(config['batch_size']):
                target, context = ta[indices], tc[indices]
                timestep = torch.randint(0, steps, (len(indices),))
                noise = torch.randn_like(target)
                alpha = alpha_bar[timestep, None]
                noisy = alpha.sqrt() * target + (1 - alpha).sqrt() * noise
                loss = nn.functional.mse_loss(model(noisy, context, timestep), noise)
                optimizer.zero_grad(set_to_none=True)
                loss.backward()
                optimizer.step()
                total += loss.item() * len(indices)
            model.eval()
            validation = 0.
            with torch.no_grad():
                for indices in torch.arange(len(va)).split(config['batch_size']):
                    alpha = alpha_bar[val_t[indices], None]
                    noisy = alpha.sqrt() * va[indices] + (1 - alpha).sqrt() * val_noise[indices]
                    loss = nn.functional.mse_loss(model(noisy, vc[indices], val_t[indices]), val_noise[indices])
                    validation += loss.item() * len(indices)
            validation /= len(va)
            selected = validation < best
            if selected:
                best = validation
                atomic_torch_save({'state_dict': model.state_dict(), 'epoch': epoch + 1, 'validation_loss': best,
                            'seed': seed, 'version': config['version'], 'action_mean': a_mean, 'action_std': a_std,
                            'context_mean': c_mean, 'context_std': c_std, 'alpha_bar': alpha_bar.numpy()}, checkpoint)
            writer.writerow({'epoch': epoch + 1, 'training_loss': total / len(ta), 'validation_loss': validation, 'selected': selected})
            handle.flush()
            atomic_torch_save({'identity': identity, 'model': model.state_dict(), 'optimizer': optimizer.state_dict(),
                'next_epoch': epoch + 1, 'best': best, 'torch_rng': torch.get_rng_state(),
                'numpy_rng': np.random.get_state(), 'python_rng': random.getstate()}, resume)
            print(f'seed={seed} epoch={epoch+1} validation={validation:.6g}', flush=True)
    saved = torch.load(checkpoint, weights_only=False)
    model.load_state_dict(saved['state_dict'])
    rng = np.random.default_rng(7811)
    example_action = rng.normal(size=(4, 360)).astype(np.float32)
    example_context = rng.normal(size=(4, 219)).astype(np.float32)
    example_t = np.array([0, 17, 63, 99], dtype=np.float32)
    with torch.no_grad():
        expected = model(torch.from_numpy(example_action), torch.from_numpy(example_context), torch.from_numpy(example_t)).numpy()
    export = {key: saved[key] for key in ['action_mean', 'action_std', 'context_mean', 'context_std', 'alpha_bar']}
    for i, layer in enumerate(model.layers, 1):
        export[f'W{i}'] = layer.weight.detach().numpy()
        export[f'b{i}'] = layer.bias.detach().numpy()[:, None]
    export.update(example_action=example_action.T, example_context=example_context.T,
                  example_t=example_t[None], example_expected=expected.T, residual_noise_skip=1, seed=seed, version=config['version'])
    mat = out / f'seed_{seed}.mat'
    temporary = mat.with_suffix('.mat.pending')
    with temporary.open('wb') as handle:
        scipy.io.savemat(handle, export, do_compression=True, oned_as='column')
    os.replace(temporary, mat)
    metadata = {'version': config['version'], 'seed': seed, 'completed': True,
        'selected_epoch': saved['epoch'], 'validation_loss': best, 'train_windows': len(ta), 'validation_windows': len(va),
        'torch_version': torch.__version__, 'checkpoint_sha256': sha256(checkpoint), 'matlab_sha256': sha256(mat),
        'epochs_completed': config['max_epochs'], 'architecture': '3 x 256 Tanh plus identity noise skip', **identity}
    temporary = metadata_path.with_suffix('.json.pending');temporary.write_text(json.dumps(metadata, indent=2), 'utf-8')
    os.replace(temporary, metadata_path)
    with attempts.open('a', encoding='utf-8') as handle:
        handle.write(json.dumps({'status': 'completed', 'utc_seconds': time.time(), 'selected_epoch': saved['epoch']}) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int)
    args = parser.parse_args()
    config = json.loads((ROOT / 'config.json').read_text('utf-8'))
    for seed in ([args.seed] if args.seed else config['training_seeds']):
        assert seed in config['training_seeds']
        train(seed, config)
