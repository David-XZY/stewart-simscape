function [provider,c,profile]=configureAbfdStaticBalance(provider,c,profile,model,referencePose)
% Explicit optional experiment input; never estimate an offset from execution data.
if ~isfield(c,'staticBalanceFile'), return; end
context=struct('model',model,'refs',struct('q',referencePose(:)));
provider=conditionResidualMeanOnStaticBalance(provider,context,profile,c.staticBalanceFile);
profile=applyAbfdStaticPreload(profile,context,c.staticBalanceFile);
c.plantOverrides.initialAppliedForce=profile.initialAppliedForce;
end
