function [next,info]=pgGuide(clean,model,task,clockHandle,budget,c)
timer=tic; next=clean; info=struct('calls',0,'backtracks',0,'accepted',false,'seconds',0);
if toc(clockHandle)>=budget, return; end
[before,g]=pgPenalty(clean.*model.outputScale(:)+model.outputMean(:),task,c); info.calls=1;
g=g.*model.outputScale(:); direction=g/max(sqrt(mean(g.^2)),1e-8);
if all(isfinite([before;direction]))
    for bt=0:3
        if toc(clockHandle)>=budget, break; end
        trial=clean-min(.1,c.guidanceEta)/2^bt*direction; info.backtracks=bt+1;
        after=pgPenalty(trial.*model.outputScale(:)+model.outputMean(:),task,c);
        if isfinite(after)&&after<before&&toc(clockHandle)<budget, next=trial; info.accepted=true; break; end
    end
end
info.seconds=toc(timer);
end
