function [U,z,info]=ch5ForceSampleV65(context,previous,w,noise)
assert(strcmp(w.parameterization,'v_force_residual')&&w.alpha_bar(end)==0,'ch5:ForceVersion','Require v-force weights.');
condition=single((context-double(w.context_mean(:)))./double(w.context_std(:)));z=single(noise);
steps=unique(round(linspace(99,0,20)),'stable');timer=tic;
for i=1:numel(steps)
    a=single(w.alpha_bar(steps(i)+1));v=ch5ForceDenoiseV65(z,condition,steps(i),w);
    clean=sqrt(a)*z-sqrt(1-a)*v;eps=sqrt(1-a)*z+sqrt(a)*v;
    if i<numel(steps),next=single(w.alpha_bar(steps(i+1)+1));z=sqrt(next)*clean+sqrt(1-next)*eps;end
end
z=double(clean);residual=double(w.force_mean(:))+double(w.force_std(:)).*z;U=reshape(residual,6,60,[])+previous;
info=struct('seconds',toc(timer),'finite',all(isfinite(U),'all'),'parameterization','v_force_residual','architecture','temporal_cnn_v64', ...
    'terminal_alpha',double(w.alpha_bar(end)),'steps',numel(steps),'candidate_count',size(U,3));
end
