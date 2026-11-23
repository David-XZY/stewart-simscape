function report=evaluateAbfdCodesignFeasibility(bundle,provider,c,design)
% Re-evaluate tightened constraints with the gain schedule actually returned.
% This supplements independent physical dense reference acceptance.
if nargin<4, design=buildAbfdSchedule(bundle.refs,provider,c); end
refs=bundle.refs; design.nominal=refs;
[cost,constraints,diagnostics]=buildAbfdTrackingExpressions([refs.q;refs.qd],refs.qdd, ...
    refs.Fleg,refs.t,provider,design,c,bundle.scene);
outputs={cost,constraints};
if isfield(diagnostics,'mean'), outputs=[outputs,{diagnostics.mean}]; end
evaluation=casadi.Function('abfd_codesign_feasibility',{},outputs);
values=evaluation.call({}); g=full(values{2});
assert(all(isfinite(g)) && isfinite(full(values{1})), ...
    'ABFD:CodesignNonfinite','Nonfinite tightened-constraint evaluation.');
[maximum,index]=max(g); rows=numel(g)/numel(refs.t);
assert(rows==floor(rows),'ABFD:CodesignConstraintShape','Unexpected constraint layout.');
report=struct('passed',maximum<=c.constraintTolerance,'maxIneq',max(0,maximum), ...
    'worstTime',refs.t(ceil(index/rows)),'worstConstraintInPoint',mod(index-1,rows)+1, ...
    'trackingCost',full(values{1}),'gainHash',abfdHash(design.schedule.K), ...
    'sampleCount',numel(refs.t),'scope','linearized tightened constraints at controller ticks and HS knots with the final gain schedule');
if isfield(diagnostics,'mean')
    report.meanDomain=validateAbfdPredictionDomain(full(values{3}),provider);
    report.passed=report.passed && report.meanDomain.maxIneq<=c.constraintTolerance;
    report.maxIneq=max(report.maxIneq,report.meanDomain.maxIneq);
    report.scope='Nonlinear predicted center and locally propagated actuator/VAR covariance at controller ticks and HS knots, with final gains; physical dense reference acceptance remains separate';
end
end
