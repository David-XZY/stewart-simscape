function result = prototype_strict_rollout(strictOverrides)
arguments
    strictOverrides struct = struct()
end
projectRoot = fullfile('D:', 'NPU', 'Matlab_code', 'stewart-simscape');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(projectRoot, 'controller', 'simscape_tracking'));
addpath(fullfile(projectRoot, 'controller', 'strict_clf_cbf_qp'));

setup = prepareSimscapePoseForceControl("");
cleanup = onCleanup(@() closePrepared(setup.modelName));
refs = setup.refs;
model = setup.model;
scene = setup.scene;
dt = setup.config.derivativeSampleTime;
time = refs.t(:).';
assert(max(abs(diff(time)-dt)) < 1e-10);

Kcontinuous = setup.design.K;
Kdiscrete = c2d(Kcontinuous, dt, 'tustin');
baseline = rolloutOne("computed-torque + LQI", false, time, refs, model, scene, setup.config, Kdiscrete, struct());
strict = rolloutOne("strict CLF-CBF-QP", true, time, refs, model, scene, setup.config, Kdiscrete, strictOverrides);
reference = struct('time', time, 'q', refs.q, 'qd', refs.qd, ...
    'qdd', refs.qdd, 'computedForce', refs.Fcomputed);
limits = struct('lengthMin',model.lmin(:),'lengthMax',model.lmax(:), ...
    'legSpeed',model.actuator.ldotMax(:),'legAcceleration',model.actuator.lddotMax(:), ...
    'forceMin',model.actuator.forceMin(:),'forceMax',model.actuator.forceMax(:), ...
    'sigmaSafe',model.singularity.sigmaMinSafe);
result = struct('reference', reference, 'limits', limits, ...
    'baseline', baseline, 'strict', strict);
fprintf('baseline passed=%d, posRMSE=%.6g, maxAcc=%.6g, minCollision=%.6g\n', ...
    baseline.metrics.passed, baseline.metrics.positionRmse, baseline.metrics.maxLegAcceleration, baseline.metrics.minCollisionDistance);
fprintf('strict passed=%d, posRMSE=%.6g, maxAcc=%.6g, minCollision=%.6g, fallback=%d, p95=%.6g\n', ...
    strict.metrics.passed, strict.metrics.positionRmse, strict.metrics.maxLegAcceleration, strict.metrics.minCollisionDistance, ...
    strict.metrics.fallbackCount, strict.metrics.qpP95);
clear cleanup;
end

function run = rolloutOne(name, useQp, time, refs, model, scene, poseConfig, Kd, strictOverrides)
N = numel(time);
dt = median(diff(time));
x = zeros(12, N);
x(:,1) = [refs.q(:,1) + [0.0004; -0.0003; 0.0002; 0.0002; -0.00015; 0.0001]; refs.qd(:,1)];
controllerState = zeros(size(Kd.A,1),1);
force = zeros(6,N);
nominal = zeros(6,N);
feedback = zeros(6,N);
legLength = zeros(6,N);
legSpeed = zeros(6,N);
legAcceleration = zeros(6,N);
sigmaMin = zeros(1,N);
sigmaLower = zeros(1,N);
collision = zeros(3,N);
cbfMargin = nan(1,N);
psi1 = nan(1,N);
hardCbfResidual = nan(1,N);
clfSlack = zeros(1,N);
qpTime = zeros(1,N);
fallback = false(1,N);
feasible = true(1,N);
activeConstraintCount = zeros(1,N);
previousForce = refs.Fcomputed(:,1);
aw = zeros(6,1);
qpConfig = makeStrictClfCbfQpConfig(model, strictOverrides);
if useQp
    % Build the exact AD backend before online timing begins.
    evaluateStrictClfCbfAdModel(x(1:6,1), x(7:12,1), model, scene, qpConfig);
    % Warm up quadprog once so reported solve times represent online steps.
    for warmupIndex = 1:5
        stepStrictClfCbfQp(x(1:6,1), x(7:12,1), refs.q(:,1), refs.qd(:,1), ...
            refs.qdd(:,1), refs.Fcomputed(:,1), previousForce, model, scene, qpConfig);
    end
end

for k = 1:N
    q = x(1:6,k); qd = x(7:12,k);
    input = [refs.q(:,k)-q; aw];
    rawFeedback = Kd.C*controllerState + Kd.D*input;
    feedback(:,k) = min(max(rawFeedback, -poseConfig.lqiFeedbackForceLimit), poseConfig.lqiFeedbackForceLimit);
    aw = feedback(:,k)-rawFeedback;
    controllerState = Kd.A*controllerState + Kd.B*[refs.q(:,k)-q; aw];
    nominal(:,k) = refs.Fcomputed(:,k)+feedback(:,k);

    qpConfig.time = time(k);
    if useQp
        [force(:,k), diagnostic] = stepStrictClfCbfQp(q, qd, refs.q(:,k), refs.qd(:,k), ...
            refs.qdd(:,k), nominal(:,k), previousForce, model, scene, qpConfig);
        cbfMargin(k) = min(diagnostic.cbf.stateMargins);
        psi1(k) = min(diagnostic.cbf.psi1);
        hardCbfResidual(k) = diagnostic.minimumCbfResidual;
        clfSlack(k) = diagnostic.clfSlack;
        qpTime(k) = diagnostic.solveTime;
        fallback(k) = diagnostic.usedFallback;
        feasible(k) = diagnostic.feasible;
        activeConstraintCount(k) = numel(diagnostic.activeConstraints);
        sigmaLower(k) = diagnostic.sigmaLowerSquared;
    else
        force(:,k) = min(max(nominal(:,k), model.actuator.forceMin), model.actuator.forceMax);
        evaluation = evaluateStrictClfCbfAdModel(q,qd,model,scene,qpConfig);
        sigmaLower(k) = evaluation.sigmaLowerSquared;
    end
    previousForce = force(:,k);

    [xdot,~] = stateDynamicsCompositeRigidBody([q;qd],force(:,k),model);
    leg = computeLegKinematics(q,qd,xdot(7:12),model);
    legLength(:,k)=leg.L; legSpeed(:,k)=leg.Ld; legAcceleration(:,k)=leg.Ldd;
    jac=sgpJacobian(q,model); sigmaMin(k)=jac.sigmaMin;
    distance=evaluateCylinderBoxDistanceNumeric(q,scene); collision(:,k)=distance.distances(:);
    if k < N
        x(:,k+1)=rk4Step(x(:,k),force(:,k),dt,model);
    end
end

poseError=x(1:6,:)-refs.q;
positionError=vecnorm(poseError(1:3,:),2,1);
attitudeError=vecnorm(poseError(4:6,:),2,1);
requiredRoof=arrayfun(@(t)evaluateStrictRoofThreshold(t,scene,qpConfig).value,time);
collisionMargin=[collision(1,:)-requiredRoof; collision(2,:)-scene.collision.safeDistance; collision(3,:)-scene.collision.safeDistance];
forceRate=[zeros(6,1),diff(force,1,2)/dt];
metrics=struct();
metrics.positionRmse=sqrt(mean(positionError.^2));
metrics.attitudeRmse=sqrt(mean(attitudeError.^2));
metrics.maxPositionError=max(positionError);
metrics.maxAttitudeError=max(attitudeError);
metrics.maxLegAcceleration=max(abs(legAcceleration),[],'all');
metrics.maxLegSpeed=max(abs(legSpeed),[],'all');
metrics.minLegLength=min(legLength,[],'all');
metrics.maxLegLength=max(legLength,[],'all');
metrics.minLegLengthMargin=min(legLength-model.lmin(:),[],'all');
metrics.maxLegLengthMargin=min(model.lmax(:)-legLength,[],'all');
metrics.maxForce=max(abs(force),[],'all');
metrics.maxForceRate=max(abs(forceRate),[],'all');
metrics.minSigma=min(sigmaMin);
metrics.minSigmaLower=sqrt(min(sigmaLower));
metrics.minCollisionDistance=min(collision,[],'all');
metrics.minCollisionMargin=min(collisionMargin,[],'all');
metrics.minCbfMargin=min(cbfMargin,[],'omitnan');
metrics.minPsi1=min(psi1,[],'omitnan');
metrics.minHardCbfResidual=min(hardCbfResidual,[],'omitnan');
metrics.maxClfSlack=max(clfSlack);
metrics.meanClfSlack=mean(clfSlack);
metrics.controlEnergy=trapz(time,sum(force.^2,1));
metrics.infeasibleCount=sum(~feasible);
metrics.fallbackCount=sum(fallback);
metrics.qpMean=mean(qpTime(qpTime>0));
metrics.qpP95=percentile(qpTime(qpTime>0),95);
metrics.qpMax=max(qpTime);
metrics.passed=metrics.maxLegAcceleration <= max(model.actuator.lddotMax)+1e-6 && ...
    metrics.maxLegSpeed <= max(model.actuator.ldotMax)+1e-6 && metrics.maxForce <= max(model.actuator.forceMax)+1e-6 && ...
    metrics.minSigma >= model.singularity.sigmaMinSafe-1e-8 && metrics.minCollisionMargin >= -1e-6 && ...
    metrics.minCbfMargin >= -1e-8 && metrics.minPsi1 >= -1e-8 && ...
    metrics.minHardCbfResidual >= -1e-7 && metrics.infeasibleCount==0 && metrics.fallbackCount==0;
run=struct('name',name,'time',time,'state',x,'force',force,'nominalForce',nominal,'feedbackForce',feedback, ...
    'legLength',legLength,'legSpeed',legSpeed,'legAcceleration',legAcceleration,'sigmaMin',sigmaMin, ...
    'sigmaLower',sigmaLower,'collisionDistance',collision,'collisionMargin',collisionMargin, ...
    'cbfMargin',cbfMargin,'psi1',psi1,'hardCbfResidual',hardCbfResidual, ...
    'clfSlack',clfSlack,'qpTime',qpTime,'fallback',fallback,'feasible',feasible, ...
    'activeConstraintCount',activeConstraintCount,'metrics',metrics);
end

function next = rk4Step(x,force,dt,model)
f=@(value) stateDerivative(value,force,model);
k1=f(x); k2=f(x+0.5*dt*k1); k3=f(x+0.5*dt*k2); k4=f(x+dt*k3);
next=x+dt*(k1+2*k2+2*k3+k4)/6;
end

function value=stateDerivative(x,force,model)
[value,~]=stateDynamicsCompositeRigidBody(x,force,model);
end

function value=percentile(data,p)
if isempty(data), value=NaN; return; end
data=sort(data(:)); index=1+(numel(data)-1)*p/100; low=floor(index); high=ceil(index);
if low==high, value=data(low); else, value=data(low)+(index-low)*(data(high)-data(low)); end
end

function closePrepared(modelName)
if bdIsLoaded(modelName), set_param(modelName,'Dirty','off'); close_system(modelName,0); end
end
