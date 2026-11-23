"""Generate independent Torch double-precision values/Jacobians for MATLAB import."""
import argparse
import hashlib
import json
from pathlib import Path

import torch

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("model"); parser.add_argument("dataset"); parser.add_argument("output")
args = parser.parse_args()
raw = Path(args.model).read_bytes(); net = json.loads(raw)
data_raw = Path(args.dataset).read_bytes(); data = json.loads(data_raw)
assert net["status"] == "trained" and net["datasetHash"] == hashlib.sha256(data_raw).hexdigest()
tensor = lambda x: torch.tensor(x, dtype=torch.float64)
xmean, xscale, yscale = map(tensor, (net["inputMean"], net["inputScale"], net["outputScale"]))
layers = [(tensor(layer["W"]), tensor(layer["b"])) for layer in net["layers"]]

def predict(x):
    h = (x-xmean)/xscale
    for index, (weight, bias) in enumerate(layers):
        h = weight @ h + bias
        if index < len(layers)-1:
            h = h.tanh()
    return h[:6]*yscale, h[6:].clamp(-12, 8).exp()*yscale.square()*net["varianceCalibration"]

indices = torch.linspace(0, len(data["x"])-1, 32).round().long().tolist()
rows = []
for index in indices:
    x = tensor(data["x"][index]).requires_grad_()
    mean, variance = predict(x)
    jacobian = torch.autograd.functional.jacobian(lambda z: predict(z)[0], x)
    rows.append({"x": x.detach().tolist(), "mean": mean.detach().tolist(),
                 "variance": variance.detach().tolist(), "meanJacobian": jacobian.tolist()})
output = {"assetHash": hashlib.sha256(raw).hexdigest(), "datasetHash": net["datasetHash"],
          "reference": "PyTorch float64 evaluation of exported trained weights", "samples": rows}
Path(args.output).write_text(json.dumps(output, allow_nan=False), encoding="utf-8")
