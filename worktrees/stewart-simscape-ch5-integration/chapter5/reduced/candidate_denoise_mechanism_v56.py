"""Fixed-window denoising diagnosis; preserves weights and production sampling."""
from pathlib import Path
import hashlib
import json
import math

import numpy as np
from scipy.io import loadmat, savemat
import torch

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'chapter5/artifacts/reduced_probe/denoise_mechanism_v56'
SOURCE = ROOT / 'chapter5/artifacts/reduced_probe/teacher_candidate_audit_v39/audit.mat'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert not OUT.exists(), 'Preserve previous diagnostic attempts.'
    OUT.mkdir()
    torch.set_num_threads(2)
    report = {'completed': False, 'source_sha256': sha(SOURCE),
              'diagnostic_source_sha256': sha(Path(__file__)), 'models': [], 'forward_noise': [],
              'sampling': [], 'selection': 'Same 24 V39 windows; four fixed common-noise candidates per model.'}
    rows = loadmat(SOURCE, simplify_cells=True)['report']['rows']
    contexts = torch.tensor(np.stack([a['context'] for a in rows]), dtype=torch.float64)
    targets = torch.tensor(np.stack([
        np.diff(np.column_stack([a['context'][30:36], a['teacher_U']]), axis=1).reshape(-1, order='F')
        for a in rows]), dtype=torch.float64)
    teacher_u = torch.tensor(np.stack([a['teacher_U'] for a in rows]), dtype=torch.float64).repeat_interleave(4, dim=0)
    previous = contexts[:, 30:36].repeat_interleave(4, dim=0)
    exports = {name: [] for name in ['original20', 'identity99_20', 'dense100']}
    seed_ids, row_ids, candidate_ids = [], [], []
    with torch.no_grad():
        for seed in range(5101, 5106):
            path = ROOT / f'chapter5/artifacts/models/seed_{seed}.mat'
            metadata = json.loads(path.with_suffix('.json').read_text('utf8'))
            assert sha(path) == metadata['matlab_sha256']
            w = loadmat(path, simplify_cells=True)
            weights = [torch.tensor(w[f'W{j}'].T, dtype=torch.float64) for j in range(1, 5)]
            biases = [torch.tensor(w[f'b{j}'], dtype=torch.float64) for j in range(1, 5)]
            frequency = torch.exp(-math.log(10000) * torch.arange(16, dtype=torch.float64) / 15)

            def denoise(z, c, t):
                ts = torch.as_tensor(t, dtype=torch.float64).reshape(-1)
                phase = ts[:, None] * frequency[None, :]
                value = torch.cat([z, c, phase.sin().expand(len(z), -1), phase.cos().expand(len(z), -1)], dim=1)
                for j in range(3):
                    value = torch.tanh(value @ weights[j] + biases[j])
                return z + value @ weights[3] + biases[3]

            example = denoise(torch.tensor(w['example_action'].T, dtype=torch.float64),
                              torch.tensor(w['example_context'].T, dtype=torch.float64), w['example_t'])
            export_error = float((example - torch.tensor(w['example_expected'].T)).abs().max())
            assert export_error < 1e-5, 'Exported denoiser reference mismatch.'
            std = torch.tensor(w['action_std'], dtype=torch.float64)
            mean = torch.tensor(w['action_mean'], dtype=torch.float64)
            c = (contexts - torch.tensor(w['context_mean'])) / torch.tensor(w['context_std'])
            target = (targets - mean) / std
            noise = torch.tensor(np.random.default_rng(9911).normal(size=target.shape))
            report['models'].append({'seed': seed, 'weight_sha256': sha(path), 'export_error': export_error,
                                     'alpha99': float(w['alpha_bar'][99]), 'action_std_median_N': float(std.median())})
            for t in [0, 25, 50, 75, 90, 95, 99]:
                alpha = float(w['alpha_bar'][t])
                noisy = math.sqrt(alpha) * target + math.sqrt(1-alpha) * noise
                predicted = denoise(noisy, c, t)
                clean = (noisy - math.sqrt(1-alpha) * predicted) / math.sqrt(alpha)
                report['forward_noise'].append({'seed': seed, 't': t,
                    'noise_rmse': float((predicted-noise).square().mean().sqrt()),
                    'increment_reconstruction_rmse_N': float(((clean-target)*std).square().mean().sqrt())})
            condition = c.repeat_interleave(4, dim=0)
            original = torch.tensor(np.random.default_rng(seed+555000).normal(size=(96, 360)), dtype=torch.float64)
            seed_ids.extend([seed] * 96)
            row_ids.extend(np.repeat(np.arange(1, 25), 4))
            candidate_ids.extend(np.tile(np.arange(1, 5), 24))
            for name, count, identity in [('original20', 20, False), ('identity99_20', 20, True), ('dense100', 100, False)]:
                steps = np.unique(np.round(np.linspace(99, 0, count)).astype(int))[::-1]
                z = original.clone()
                for i, t in enumerate(steps):
                    alpha = float(w['alpha_bar'][t])
                    predicted = z if identity and i == 0 else denoise(z, condition, t)
                    clean = (z - math.sqrt(1-alpha) * predicted) / math.sqrt(alpha)
                    if i+1 < len(steps):
                        nxt = float(w['alpha_bar'][steps[i+1]])
                        z = math.sqrt(nxt) * clean + math.sqrt(1-nxt) * predicted
                raw_delta = clean * std + mean
                exports[name].append(raw_delta.numpy().T)
                delta = raw_delta.reshape(96, 60, 6).transpose(1, 2)
                u = previous[:, :, None] + delta.cumsum(dim=2)
                last = previous.clone()
                for j in range(60):
                    u[:, :, j] = torch.minimum(torch.maximum(u[:, :, j], torch.maximum(last-160, torch.full_like(last, -2000))),
                                              torch.minimum(last+160, torch.full_like(last, 2000)))
                    last = u[:, :, j].clone()
                error = u - teacher_u
                increment_error = torch.diff(torch.cat([torch.zeros((96, 6, 1)), error], dim=2), dim=2)
                report['sampling'].append({'seed': seed, 'variant': name,
                    'force_error_median_N': float(np.median(error.square().mean(dim=(1, 2)).sqrt().numpy())),
                    'increment_error_median_N': float(np.median(increment_error.square().mean(dim=(1, 2)).sqrt().numpy()))})
            print(f'Seed {seed}: fixed-noise sampling and seven training-noise levels completed.', flush=True)
    data = {key: np.concatenate(value, axis=1) for key, value in exports.items()}
    data.update(row_ids=np.asarray(row_ids), seed_ids=np.asarray(seed_ids), candidate_ids=np.asarray(candidate_ids))
    savemat(OUT / 'candidates.mat', data, do_compression=True, oned_as='row')
    report['candidates_sha256'] = sha(OUT / 'candidates.mat')
    report['completed'] = True
    (OUT / 'neural_summary.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps(report['sampling']), flush=True)


if __name__ == '__main__':
    main()
