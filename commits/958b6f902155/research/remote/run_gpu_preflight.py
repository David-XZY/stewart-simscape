"""CUDA execution check only; does not create training data or model weights."""
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np
import torch


def main():
    torch.manual_seed(20260905)
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA is required; CPU fallback is disabled')
    device = torch.device('cuda:0')
    model = torch.nn.Sequential(torch.nn.Linear(12, 64), torch.nn.Tanh(),
                                torch.nn.Linear(64, 64), torch.nn.Tanh(),
                                torch.nn.Linear(64, 12)).to(device)
    x = torch.randn(2048, 12, device=device)
    target = torch.sin(x)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-3)
    losses = []
    torch.cuda.synchronize()
    start = time.perf_counter()
    for _ in range(30):
        optimizer.zero_grad(set_to_none=True)
        loss = (model(x) - target).square().mean()
        loss.backward()
        if not all(torch.isfinite(p.grad).all().item() for p in model.parameters()):
            raise RuntimeError('Nonfinite CUDA gradients')
        optimizer.step()
        losses.append(loss.detach().item())
    torch.cuda.synchronize()
    if not losses[-1] < losses[0]:
        raise RuntimeError('CUDA optimizer check did not reduce loss')
    report = dict(scope='synthetic CUDA environment check; not formal training',
                  passed=True, python=platform.python_version(), numpy=np.__version__,
                  torch=torch.__version__, cuda=torch.version.cuda,
                  gpu=torch.cuda.get_device_name(0), device=str(device),
                  seconds=time.perf_counter()-start, loss_first=losses[0],
                  loss_last=losses[-1], steps=len(losses))
    path = Path(sys.argv[1])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
