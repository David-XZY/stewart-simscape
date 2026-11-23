function [cp,info]=prepareGeometricSeed(task,index,c)
% Traditional (nonlearned) multistart preparation, included in offline/online time.
timer=tic; cp=dpSeed(task,index); info=struct('usedProjection',false,'reason','','seconds',0);
try
    report=dpValidate(struct('kind','spline','cp',cp,'T',task.T),task,c);
    if ~report.passed
        [candidate,projected,details]=dpQuickFit(task,cp,c);
        info.projection=details;
        if projected.passed, cp=candidate; info.usedProjection=true; end
    end
catch e
    % Retain and try the original start; failed preprocessing cannot hide an attempt.
    info.reason=e.message;
end
info.seconds=toc(timer);
end
