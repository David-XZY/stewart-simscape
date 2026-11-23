function report=calibrateAbfdStaticPreload(bundle,c,profile,outputDir)
% Measure the force-to-initial-acceleration map in the existing Simscape plant.
% This artifact only initializes the plant; it is not a controller or B label.
assert(~isfield(profile,'initialAppliedForce'),'ABFD:PreloadAlreadySet','Calibrate an unmodified plant profile.');
assert(norm(bundle.refs.qd(:,1),inf)<1e-10,'ABFD:StaticPreloadVelocity','Static calibration needs zero initial velocity.');
assert(~isfile(fullfile(outputDir,'status.json')),'ABFD:CalibrationExists','Use a fresh calibration output.');
if ~isfolder(outputDir), mkdir(outputDir); end
report=struct('state','running','passed',false,'scope','Static initialization calibration; no closed-loop performance acceptance');
writeAbfdJson(fullfile(outputDir,'status.json'),report);
try
    p=buildDynamicsProvider(bundle.model);
    setup=prepareAbfdSimscape(bundle,p,c,profile); cleanup=onCleanup(@()close_system(setup.modelName,0));
    sensor=installAbfdAccelerationProbe(setup.modelName); set_param(setup.modelName,'StopTime','.0001');
    stewart=evalin('base','stewart'); nominal=setup.runtime.initialAppliedForce; delta=10;
    measured=zeros(6,7); forces=zeros(6,7);
    for k=1:7
        applied=nominal; if k>1, applied(k-1)=applied(k-1)+delta; end
        [measured(:,k),forces(:,k)]=measure(applied);
        writeAbfdJson(fullfile(outputDir,'sensitivity_progress.json'), ...
            struct('completed',k,'forces',forces(:,1:k),'initialSpatialAcceleration',measured(:,1:k)));
    end
    sensitivity=(measured(:,2:7)-measured(:,1))/delta;
    assert(rcond(sensitivity)>1e-8,'ABFD:PreloadSensitivity','Force sensitivity is ill-conditioned.');
    equilibrium=nominal-sensitivity\measured(:,1);
    [residual,actual]=measure(equilibrium);
    assert(norm(residual,inf)<1e-5,'ABFD:StaticEquilibrium','Measured preload is not a static equilibrium.');
    report=struct('state','complete','passed',true,'nominalForce',nominal,'equilibriumForce',actual, ...
        'initialAccelerationBefore',measured(:,1),'initialAccelerationAfter',residual,'sensitivity',sensitivity, ...
        'pose',bundle.refs.q(:,1)+profile.initialPoseOffset,'plantHash',abfdHash(profile), ...
        'modelHash',abfdHash(bundle.model), ...
        'scope','Measured static initial equilibrium; plant-specific, shared by paired controller comparisons');
    save(fullfile(outputDir,'calibration.mat'),'report','sensor','profile','c');
    writeAbfdJson(fullfile(outputDir,'status.json'),report); clear cleanup;
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message;
    writeAbfdJson(fullfile(outputDir,'status.json'),report); rethrow(e);
end
    function [acceleration,actual]=measure(applied)
        assert(all(isfinite(applied)) && all(applied>=bundle.model.actuator.forceMin & ...
            applied<=bundle.model.actuator.forceMax),'ABFD:InitialForceBounds','Calibration force exceeds actuator limits.');
        stewart.nonidealForceActuator.initialForce=applied; assignin('base','stewart',stewart);
        simulation=sim(setup.modelName,'ReturnWorkspaceOutputs','on');
        direct=simulation.get('abfdSpatialAcceleration'); values=squeeze(direct.Data);
        if size(values,1)~=numel(direct.Time), values=values.'; end
        simout=simulation.get('simout'); forceValues=squeeze(simout.y.Taum.Data);
        if size(forceValues,1)~=numel(simout.y.Taum.Time), forceValues=forceValues.'; end
        actual=forceValues(1,:).'; acceleration=values(1,:).';
        assert(norm(actual-applied)<1e-8,'ABFD:PreloadActualForce','Requested force was not physically applied.');
        assert(all(isfinite(acceleration)),'ABFD:PreloadAcceleration','Invalid measured acceleration.');
    end
end
