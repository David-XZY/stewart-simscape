function report=run_A0_trackability_metric(refs,provider,c,design)
assert(any(strcmp(c.controllerMode,{'fullstate-tvlqr','geometric-D'})), ...
    'ABFD:PredictionController','A0 requires the actual TVLQR or D feedback law.');
if nargin<4 || isempty(design), design=buildAbfdSchedule(refs,provider,c); end
design.nominal=refs;
[value,~,diag]=buildAbfdTrackingExpressions([refs.q;refs.qd],refs.qdd, ...
    refs.Fleg,refs.t,provider,design,c,struct('hood',struct('obstacles',[])));
outputs=[{value},diag.P]; count=numel(diag.P);
if isfield(diag,'mean'), outputs=[outputs,{diag.mean}]; end
fun=casadi.Function('abfd_metric_value',{},outputs); output=fun.call({});
report=struct('predictedCost',full(output{1}),'design',design,'P',{cellfun(@full,output(2:count+1),'UniformOutput',false)}, ...
    'status','prediction-only; empirical ranking pending experiments');
if isfield(diag,'predictionModel'), report.predictionModel=diag.predictionModel; end
if isfield(diag,'mean')
    report.mean=full(output{count+2});
    report.meanDomain=validateAbfdPredictionDomain(report.mean,provider);
end
end
