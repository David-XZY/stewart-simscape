import numpy as np
import torch
from pg_core import read,write
from pg_torch import trajectory_constraint_loss

def audit(path,out):
    o=read(path); z=torch.tensor(o['free'],dtype=torch.float64,requires_grad=True)
    value,groups=trajectory_constraint_loss(z,o['task'],o['envelope']); grad=torch.autograd.grad(value,z)[0]
    vg=np.array([float(x.detach()) for x in groups.values()]); expected=np.asarray(o['groups'])
    np.testing.assert_allclose(vg,expected,rtol=1e-6,atol=1e-8)
    reference=np.array(o['gradient']); error=float(np.max(np.abs(grad.numpy()-reference))/max(1,np.max(np.abs(reference))))
    assert error<1e-3,error
    result=dict(passed=True,value=float(value.detach()),matlabValue=o['value'],groups=vg.tolist(),gradientRelative=error)
    write(out,result); print(result)
