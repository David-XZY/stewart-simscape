function scene=sampleFeasibleStartPose(scene,model,seed)
% Rejection sampling changes an invalid fixed start, never the path limits.
stream=RandStream('mt19937ar','Seed',seed+2000000);
original=scene.q0; rejected=struct('lengthViolation',{},'sigmaMin',{},'clearance',{});
required=scene.collision.safeDistance*ones(numel(scene.hood.obstacles),1);
required(1)=scene.collision.stage1ConstraintDistance;
for attempt=0:63
    if attempt>0
        scene.q0=model.qHome+[.04*(rand(stream,3,1)-.5);deg2rad(6)*(rand(stream,3,1)-.5)];
    end
    kin=sgpIK(scene.q0,model); J=sgpJacobian(scene.q0,model);
    violation=max([kin.L-model.lmax;model.lmin-kin.L]);
    clearance=[];
    if violation<=0 && J.sigmaMin>=model.singularity.sigmaMinSafe
        collision=evaluateCylinderBoxDistanceNumeric(scene.q0,scene);
        clearance=min(collision.distances(:)-required);
        if clearance>=0
            if attempt>0
                scene.initialPoseResampling=struct('version',1,'originalPose',original, ...
                    'attempts',attempt,'rejected',rejected);
            end
            return;
        end
    end
    rejected(end+1)=struct('lengthViolation',violation,'sigmaMin',J.sigmaMin,'clearance',clearance); %#ok<AGROW>
end
error('ABFD:NoFeasibleStartPose','No valid fixed start after 64 deterministic samples.');
end
