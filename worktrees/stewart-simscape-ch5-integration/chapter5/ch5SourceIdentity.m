function identity=ch5SourceIdentity()
[c,root]=ch5Init();files=["ch5Physics.m","ch5AbsoluteState.m","ch5Rotation.m", ...
    "ch5CollisionModel.m","ch5SafetyQp.m","ch5Ihsid.m","ch5PlanPostcheck.m","ch5Reference.m", ...
    "ch5DiscreteLqi.m","ch5PrepareSimscape.m","ch5SimscapeSfun.m","ch5PlantModel.m","ch5Observe.m","ch5ActuatorProfile.m","ch5GjkDistance.m","ch5Metrics.m","ch5GenerateTasks.m","ch5JitOptions.m","compile.py"];
digest=java.security.MessageDigest.getInstance('SHA-256');individual=struct();
keys={'version','Ts','internal_step','H','episode_seconds','replan_seconds','max_obstacles','safe_distance','sample_hold_margin','lqi_control_scale', ...
    'actuator_slew_rate','actuator_time_constant','actuator_delay','actuator_deadzone','native_mixed_optimization','native_function_optimization_bytes', ...
    'position_tolerance','rotation_tolerance_degrees','linear_speed_tolerance','angular_speed_tolerance_degrees','arrival_hold_seconds','position_tight','rotation_tight_degrees'};
baselineConfig=struct();for j=1:numel(keys),baselineConfig.(keys{j})=c.(keys{j});end
digest.update(unicode2native(jsonencode(baselineConfig),'UTF-8'));
for name=files
    file=fullfile(root,'chapter5',name);fid=fopen(file,'rb');bytes=fread(fid,inf,'*uint8');fclose(fid);
    one=java.security.MessageDigest.getInstance('SHA-256');one.update(bytes);
    value=lower(reshape(dec2hex(typecast(one.digest(),'uint8'),2).',1,[]));
    individual.(matlab.lang.makeValidName(name))=value;digest.update(bytes);
end
identity=struct('baseline_source_sha256',lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[])), ...
    'files',individual,'baseline_commit','ae06b7508151aea2c704d4b1a204bd9fb45e3347');
% Private predictive-control fixes must not invalidate baseline teachers,
% but they must invalidate cached diffusion episodes before the freeze.
newFiles=["ch5DiffusionControl.m","ch5PredictiveOptimizer.m","ch5SolveProjection.m", ...
    "ch5InspectCandidate.m","ch5ControlInitialGuess.m","ch5DerivativeMap.m","ch5IntervalConstraints.m","ch5NativeNodes.m","ch5NativePreview.m", ...
    "ch5CheckRollout.m","ch5PredictionAccelerationLimit.m","ch5NativeCollisionAudit.m","ch5Denoise.m","ch5Context.m", ...
    "ch5ProjectionJitOptions.m","compile_projection.py","child_job.py","ch5SourceIdentity.m"];
newDigest=java.security.MessageDigest.getInstance('SHA-256');
newDigest.update(unicode2native(identity.baseline_source_sha256,'UTF-8'));
newDigest.update(unicode2native(jsonencode(c),'UTF-8'));
for name=newFiles
    fid=fopen(fullfile(root,'chapter5',name),'rb');assert(fid>=0);bytes=fread(fid,inf,'*uint8');fclose(fid);
    newDigest.update(bytes);
end
identity.diffusion_source_sha256=lower(reshape(dec2hex(typecast(newDigest.digest(),'uint8'),2).',1,[]));
end
