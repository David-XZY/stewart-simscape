function profile=applyAbfdStaticPreload(profile,bundle,file)
% A measured plant initialization is shared across paired controller models.
calibration=readAbfdJsonUtf8(file);
assert(calibration.passed && strcmp(calibration.state,'complete'), ...
    'ABFD:UnacceptedPreload','Static preload calibration was not completed.');
baseProfile=profile;
if isfield(baseProfile,'initialAppliedForce'), baseProfile=rmfield(baseProfile,'initialAppliedForce'); end
assert(strcmp(calibration.plantHash,abfdHash(baseProfile)) && ...
    strcmp(calibration.modelHash,abfdHash(bundle.model)) && ...
    norm(calibration.pose(:)-bundle.refs.q(:,1)-profile.initialPoseOffset,inf)<1e-10, ...
    'ABFD:PreloadContext','Preload belongs to a different plant, model or initial pose.');
assert(norm(calibration.initialAccelerationAfter,inf)<1e-5,'ABFD:PreloadResidual','Measured initial acceleration was not zero.');
force=calibration.equilibriumForce(:);
validateattributes(force,{'double'},{'size',[6,1],'real','finite'});
if isfield(profile,'initialAppliedForce')
    assert(isequal(profile.initialAppliedForce,force),'ABFD:PreloadInitialForce', ...
        'An explicit physical initial force must exactly match the independent observation.');
end
assert(all(force>=bundle.model.actuator.forceMin & force<=bundle.model.actuator.forceMax), ...
    'ABFD:InitialForceBounds','Static preload exceeds physical actuator limits.');
profile.initialAppliedForce=force;
end
