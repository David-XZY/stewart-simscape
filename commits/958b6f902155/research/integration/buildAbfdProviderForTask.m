function [provider,c,profile]=buildAbfdProviderForTask(model,c,profile,referencePose)
% Apply only explicitly supplied pre-task observations; no execution-log fit.
startup=[];
if isfield(c,'plantInitializationFile')
    context=struct('model',model,'refs',struct('q',referencePose(:)));
    profile=applyAbfdStaticPreload(profile,context,c.plantInitializationFile);
    c.plantOverrides.initialAppliedForce=profile.initialAppliedForce;
end
if isfield(c,'inertiaCalibrationFile')
    context=struct('model',model,'refs',struct('q',referencePose(:)));
    assert(strcmp(abfdFileHash(c.inertiaCalibrationFile),abfdFileHash(c.staticBalanceFile)), ...
        'ABFD:StartupBalanceMismatch','Inertia and static balance must refer to the same observation.');
    startup=readMeasuredInertiaRegression(c.inertiaCalibrationFile,context,profile);
end
provider=buildDynamicsProvider(model,c.modelMode,c.residualFile,c.allowTestFixtures, ...
    referencePose(:)+profile.initialPoseOffset,startup);
[provider,c,profile]=configureAbfdStaticBalance(provider,c,profile,model,referencePose);
end
