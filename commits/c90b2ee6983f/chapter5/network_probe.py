"""Nonzero random weights check the export API, not trained model quality."""
from pathlib import Path
import numpy as np
import scipy.io
import torch
from train import Denoiser

torch.manual_seed(9821)
model = Denoiser(360, 219, [256, 256, 256])
for parameter in model.parameters():
    torch.nn.init.normal_(parameter, std=.02)
rng = np.random.default_rng(9821)
action = rng.normal(size=(4, 360)).astype(np.float32)
context = rng.normal(size=(4, 219)).astype(np.float32)
t = np.array([0, 17, 63, 99], dtype=np.float32)
with torch.no_grad():
    expected = model(torch.from_numpy(action), torch.from_numpy(context), torch.from_numpy(t)).numpy()
export = {'example_action': action.T, 'example_context': context.T, 'example_t': t[None],
          'example_expected': expected.T, 'residual_noise_skip': 1, 'trained': False}
for i, layer in enumerate(model.layers, 1):
    export[f'W{i}'] = layer.weight.detach().numpy();export[f'b{i}'] = layer.bias.detach().numpy()[:, None]
path = Path(__file__).resolve().parent / 'artifacts' / 'validation' / 'network_api_probe.mat'
scipy.io.savemat(path, export, do_compression=True)
print('Saved untrained API probe in validation directory; no model checkpoint was created.')
