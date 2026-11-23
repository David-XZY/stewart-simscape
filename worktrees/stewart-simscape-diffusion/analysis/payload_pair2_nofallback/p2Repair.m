function [bundle,info]=p2Repair(bundle,task,clockHandle,budget,c)
% Original pgRepair arithmetic, with explicit continuation/termination evidence.
timer=tic; assert(strcmp(bundle.kind,'spline'),'P2:Representation','Spline required.');
n=c.controlCount; cp=bundle.cp; z=reshape(cp(:,4:end-3),[],1);
scale=repmat([.1;.1;.1;.2;.2;.2],n-6,1); accepted=0; iterations=0;
reason='iteration_block'; first=NaN; last=NaN;
for it=1:40
    if toc(clockHandle)>=budget, reason='deadline'; break; end
    [value,g]=pgPenalty(z,task,c); iterations=it;
    if it==1, first=value; end
    last=value; direction=g.*scale; direction=direction/max(sqrt(mean(direction.^2)),1e-8);
    if any(~isfinite([value;direction])), reason='invalid_gradient'; break; end
    improved=false;
    for bt=0:3
        if toc(clockHandle)>=budget, reason='deadline'; break; end
        trial=z-.03/2^bt*scale.*direction; val=pgPenalty(trial,task,c);
        if val<value && toc(clockHandle)<budget
            z=trial; accepted=accepted+1; last=val; improved=true; break;
        end
    end
    if ~improved
        if toc(clockHandle)>=budget, reason='deadline'; else, reason='line_search_stagnant'; end
        break;
    end
end
cp(:,4:end-3)=reshape(z,6,n-6); bundle=struct('kind','spline','T',task.T,'cp',cp);
info=struct('iterations',iterations,'acceptedSteps',accepted,'seconds',toc(timer), ...
    'reason',reason,'continueAllowed',strcmp(reason,'iteration_block')&&iterations==40&&accepted>0&&toc(clockHandle)<budget, ...
    'firstPenalty',first,'lastPenalty',last,'withinBudget',toc(clockHandle)<=budget);
end
