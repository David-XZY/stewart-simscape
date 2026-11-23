function [U,z,info]=ch5ForceSampleV76(context,previous,w,noise,r,k)
assert(isfield(w,'context_supported'),'ch5:ConditionMask','Require training-coverage mask.');
[U,z,info]=ch5ForceSampleV74(context,previous,w,noise,r,k);
unsupported=~logical(w.context_supported(:));delta=context(:)-double(w.context_mean(:));
info.unsupported_condition_indices=find(unsupported&abs(delta)>1e-8);info.unobserved_conditions_present=~isempty(info.unsupported_condition_indices);
info.unobserved_conditions_learned=false;info.conditioning_revision='constant-column-neutralization-v76';
end
