function [U,z,info]=ch5ForceSampleV78(context,previous,w,noise,r,k)
% The latent sequence induces DeltaU through exact augmented-state shooting.
assert(strcmp(w.parameterization,'v_feedback_innovation')&&w.alpha_bar(end)==0,'ch5:ForceVersion','Require innovation weights.');
timer=tic;condition=single((context-double(w.context_mean(:)))./double(w.context_std(:)));z=single(noise);assert(isfield(w,'ddim_timesteps')&&w.ddim_steps==10,'ch5:SamplerSchedule','Require a checkpoint trained with ten-step sampling.');
steps=double(w.ddim_timesteps(:)');assert(isequal(steps,99:-11:0),'ch5:SamplerSchedule','MATLAB and training timesteps differ.');
for i=1:numel(steps)
    a=single(w.alpha_bar(steps(i)+1));v=ch5ForceDenoiseV65(z,condition,steps(i),w);
    clean=sqrt(a)*z-sqrt(1-a)*v;eps=sqrt(1-a)*z+sqrt(a)*v;
    if i<numel(steps),next=single(w.alpha_bar(steps(i+1)+1));z=sqrt(next)*clean+sqrt(1-next)*eps;end
end
z=double(clean);V=reshape(double(w.force_mean(:))+double(w.force_std(:)).*z,6,60,[]);inference=toc(timer);
term=ch5ForceTerminalV62(context(1:6),r,k);L=term.Du*term.K./term.scale';decoder=ch5ForceInnovationV73(r);B=size(V,3);
mapped=decoder.map(B);U=full(mapped(repmat(context(1:r.plant.n),1,B),repmat(previous,1,B),reshape(V,6,[]),repmat(L,1,B),repmat(term.xi,1,B)));U=reshape(U,6,60,B);
info=struct('seconds',toc(timer),'network_seconds',inference,'finite',all(isfinite(U),'all'),'parameterization','v_feedback_innovation', ...
    'steps',numel(steps),'candidate_count',B,'projection',false,'tail_repair',false);
unsupported=~logical(w.context_supported(:));delta=context(:)-double(w.context_mean(:));
info.unsupported_condition_indices=find(unsupported&abs(delta)>1e-8);
info.unobserved_conditions_present=~isempty(info.unsupported_condition_indices);info.unobserved_conditions_learned=false;
end
