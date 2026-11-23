function [proposed,qsNew,info,warm]=ch5ForceSolveStepV77(prepared,prior,warm)
% Solve only the requested candidate against the unchanged shared QP.
if nargin<3,warm=[];end
difference=prior-prepared.first_prior;
f=prepared.base_f-prepared.prior_gradient*difference(:);
[delta,info,warm]=ch5ForceQpSolveV62(prepared.qp,f,warm);
info.linearization_phases=prepared.linearization_phases;info.sensitivity=prepared.sensitivity;
info.terminal_region_certified=false;info.terminal_region_rho=prepared.terminal_rho;
info.variables=54;info.starts=[1,6,16,26,36,46,51,56];info.shared_linearization=true;
info.terminal_approximation='inscribed_metric_box';proposed=prepared.U;qsNew=prepared.qs;
if info.finite
    info.predicted_task_cost=prepared.task_base+prepared.ftask'*delta+.5*delta'*prepared.Qtask*delta;
    proposed=full(prepared.feedback(prepared.x,prepared.nodes,prepared.U,prepared.K,reshape(delta(1:48),6,8)));
    qsNew=prepared.qs+prepared.Pq*delta(49:54);
end
end
