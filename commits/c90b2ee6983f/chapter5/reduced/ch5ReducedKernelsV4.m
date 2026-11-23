function k=ch5ReducedKernelsV4(r)
% Preserve all primal/checking kernels; factor only the exact nonideal Jacobian.
k=ch5ReducedKernels(r);
if r.mode=="nonideal-force"&&isfield(r.reduced,'factor_derivatives')&&r.reduced.factor_derivatives
    stage=ch5ReducedFactorStage(r);k.stage=stage;k.stages=stage.map(r.c.H);
end
end
