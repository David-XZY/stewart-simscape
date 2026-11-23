function [bundle,info]=p474Repair(bundle,task,clock,budget,c)
% A continuation block. No IHSID calls and no objective-quality gate.
timer=tic;n=c.controlCount;
if strcmp(bundle.kind,'spline'),cp=bundle.cp;
else
    t=linspace(0,task.T,201);tr=bdReconstruct(bundle,t);[B,~,~]=bdBasis(t,task.T,n);
    cp=bdBaseline(task,n);cp(:,4:end-3)=0;cp(:,4:end-3)=(tr.q-cp*B)/B(4:end-3,:);
end
z=reshape(cp(:,4:end-3),[],1);scale=repmat([.1;.1;.1;.2;.2;.2],n-6,1);accepted=0;reason='iteration_block';it=0;
for it=1:40
    if toc(clock)>=budget,reason='time_slice';break;end
    [v,g]=pgPenalty(z,task,c);direction=g.*scale;direction=direction/max(sqrt(mean(direction.^2)),1e-8);
    if any(~isfinite([v;direction])),reason='invalid_candidate_gradient';break;end
    improved=false;
    for bt=0:3
        if toc(clock)>=budget,reason='time_slice';break;end
        trial=z-.03/2^bt*scale.*direction;nv=pgPenalty(trial,task,c);
        if isfinite(nv)&&nv<v&&toc(clock)<budget,z=trial;accepted=accepted+1;improved=true;break;end
    end
    if ~improved
        if ~strcmp(reason,'time_slice'),reason='line_search_stagnant';end
        break;
    end
end
cp(:,4:end-3)=reshape(z,6,n-6);bundle=struct('kind','spline','cp',cp,'T',task.T);
info=struct('iterations',it,'acceptedSteps',accepted,'reason',reason,'exhausted',any(strcmp(reason,{'line_search_stagnant','invalid_candidate_gradient'})),'seconds',toc(timer));
end
