function [cp,report,info]=dpQuickFit(task,initial,c)
% Low-cost physical projection before the full constrained fit fallback.
% This does not use a learned model and never substitutes for dense acceptance.
timer=tic; base=dpBaseline(task);
asset=struct('outputMean',reshape(initial(:,4:13)-base(:,4:13),[],1), ...
    'outputScale',repmat([.1;.1;.1;.2;.2;.2],10,1));
local=c; local.guidanceWeights=[10,1,1,0,0]; fun=dpGuidance(task,asset,local);
z=zeros(60,1); cp=initial; report=struct('passed',false); acceptedSteps=0;
for iteration=1:40
    [cost,grad]=fun(z); cost=full(cost); grad=full(grad); accepted=false;
    if ~isfinite(cost) || ~all(isfinite(grad)), break; end
    direction=grad/max(1e-12,sqrt(mean(grad.^2)));
    for backtrack=0:8
        trial=z-.02/(2^backtrack)*direction;
        if sqrt(mean(trial.^2))>.25, continue; end
        value=full(fun(trial));
        if isfinite(value) && value<cost, z=trial; accepted=true; acceptedSteps=acceptedSteps+1; break; end
    end
    if mod(iteration,5)==0 || ~accepted || iteration==40
        cp=base; cp(:,4:13)=base(:,4:13)+reshape(asset.outputMean+asset.outputScale.*z,6,10);
        report=dpValidate(struct('kind','spline','cp',cp,'T',task.T),task,c);
        if report.passed || ~accepted, break; end
    end
end
info=struct('passed',report.passed,'iterations',iteration,'acceptedSteps',acceptedSteps, ...
    'normalizedChangeRms',sqrt(mean(z.^2)),'seconds',toc(timer), ...
    'method','bounded physical gradient projection; independent dense acceptance');
end
