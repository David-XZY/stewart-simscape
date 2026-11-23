function [bundle,info]=pgRepair(bundle,task,clockHandle,budget,c)
timer=tic; n=c.controlCount;
if strcmp(bundle.kind,'spline'), cp=bundle.cp;
else
    times=linspace(0,task.T,201); tr=bdReconstruct(bundle,times); [B,~,~]=bdBasis(times,task.T,n);
    cp=bdBaseline(task,n); cp(:,4:end-3)=0; cp(:,4:end-3)=(tr.q-cp*B)/B(4:end-3,:);
end
z=reshape(cp(:,4:end-3),[],1); scale=repmat([.1;.1;.1;.2;.2;.2],n-6,1); accepted=0; it=0;
for it=1:40
    if toc(clockHandle)>=budget, break; end
    [value,g]=objective(z); direction=g.*scale; direction=direction/max(sqrt(mean(direction.^2)),1e-8);
    if any(~isfinite([value;direction])), break; end
    improved=false;
    for bt=0:3
        if toc(clockHandle)>=budget, break; end
        trial=z-.03/2^bt*scale.*direction; val=objective(trial);
        if val<value && toc(clockHandle)<budget, z=trial; accepted=accepted+1; improved=true; break; end
    end
    if ~improved, break; end
end
cp(:,4:end-3)=reshape(z,6,n-6); bundle=struct('kind','spline','T',task.T,'cp',cp);
info=struct('iterations',it,'acceptedSteps',accepted,'seconds',toc(timer),'withinBudget',toc(clockHandle)<=budget);
    function [val,grad]=objective(z)
        [val,grad]=pgPenalty(z,task,c);
        if strcmp(c.returnPolicy,'quality')
            % Analytic detour objective gradients use fixed endpoint scale and
            % actual spatial angular velocity, not Euler derivative magnitude.
            [qval,qgrad]=pgDetourPenalty(z,task,c); val=val+qval; grad=grad+qgrad;
        end
    end
end
