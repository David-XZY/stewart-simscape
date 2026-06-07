function report = validateTrajectoryDenseComparison(method, traj, data, model, scene, disc)
% validateTrajectoryDenseComparison - 三种隐式动力学方法的统一后验验证接口
if strcmp(method, 'DMSID')
    report = validateDMSIDDense(traj, data, model, scene, disc);
else
    report = validateTrajectoryDenseImplicit(traj, model, scene, disc);
    report.trajectoryQualityDiagnostics = struct( ...
        'maxDenseDynResidualGeometric', report.maxDynResidualGeometric, ...
        'maxDenseKinematicResidual', report.maxKinematicResidual, ...
        'note', 'CHS/IHS dense 动力学残差是配点插值质量诊断，不直接作为工程失败条件。');
    report.engineeringTrajectoryPassed = report.pathPassed && report.forcePassed && ...
        report.singularityPassed && report.stage2Passed && report.insertionContinuousPassed;
end
if isfield(data, 'maxDefectResidual')
    report.maxDefectResidual = data.maxDefectResidual;
else
    report.maxDefectResidual = maxDefectFromHS(data, disc);
end
report.method = method;
end

function maxDefect = maxDefectFromHS(data, disc)
values = zeros(12, disc.numIntervals);
for intervalIndex = 1:disc.numIntervals
    values(:, intervalIndex) = data.Xnode(:, intervalIndex+1) - data.Xnode(:, intervalIndex) - ...
        disc.h/6*(data.fNode(:, intervalIndex) + 4*data.fMid(:, intervalIndex) + data.fNode(:, intervalIndex+1));
end
maxDefect = max(abs(values(:)));
end

function report = validateDMSIDDense(traj, data, model, scene, disc)
% validateDMSIDDense - DMSID 的 A 插值积分后验验证
%
% DMSID 的动力学表达为隐式形式，dense 点使用 A_k、A_c、A_{k+1}
% 二次插值得到的 A(t) 计算腿加速度和动力学残差。
% “由 F 反解 A”不同，但工程约束口径保持一致。
samplesPerInterval = 20;
allC = [];
allLength = [];
allLd = [];
allLdd = [];
allForce = [];
allSigma = [];
allCond = [];
allClearance = [];
allObstacleClearance = zeros(numel(scene.hood.obstacles), 0);
allStage1Clearance = [];
allStage1ObstacleClearance = zeros(numel(scene.hood.obstacles), 0);
stage2SideClearance = [];
allDyn = [];
allTime = [];
stage2LateralError = [];
stage2HeightError = [];
stage2AttitudeError = [];
stage2InsertionSpeed = [];

for intervalIndex = 1:disc.numIntervals
    for subIndex = 0:samplesPerInterval
        if intervalIndex > 1 && subIndex == 0
            continue;
        end
        tau = subIndex / samplesPerInterval;
        X = integrateImplicitAToTau(data.Xnode(:, intervalIndex), data.Anode(:, intervalIndex), ...
            data.Amid(:, intervalIndex), data.Anode(:, intervalIndex+1), tau, disc.h);
        A = quadraticAccel(data.Anode(:, intervalIndex), data.Amid(:, intervalIndex), ...
            data.Anode(:, intervalIndex+1), tau);
        F = quadraticForce(data.Fnode(:, intervalIndex), data.Fmid(:, intervalIndex), ...
            data.Fnode(:, intervalIndex+1), tau);
        point = evaluatePathConstraintsAtPoint(X, F, model, scene, A);
        allC = [allC; point.cPath]; %#ok<AGROW>
        allLength = [allLength, point.L]; %#ok<AGROW>
        allLd = [allLd, point.Ld]; %#ok<AGROW>
        allLdd = [allLdd, point.Ldd]; %#ok<AGROW>
        allForce = [allForce, F]; %#ok<AGROW>
        allSigma = [allSigma, point.sigmaMin]; %#ok<AGROW>
        allCond = [allCond, point.condJ]; %#ok<AGROW>
        clearanceVector = point.collisionDistances(:);
        allClearance = [allClearance; clearanceVector]; %#ok<AGROW>
        allObstacleClearance = [allObstacleClearance, clearanceVector]; %#ok<AGROW>
        sampleTime = disc.tNode(intervalIndex) + tau*disc.h;
        allTime = [allTime, sampleTime]; %#ok<AGROW>
        if sampleTime <= disc.durationApproach + 1e-12
            allStage1Clearance = [allStage1Clearance; clearanceVector]; %#ok<AGROW>
            allStage1ObstacleClearance = [allStage1ObstacleClearance, clearanceVector]; %#ok<AGROW>
        end
        allDyn = [allDyn, point.dynAux.rDyn]; %#ok<AGROW>
        if sampleTime >= disc.durationApproach - 1e-12
            clearGeom = evaluateCylinderBoxClearance(X(1:6), scene);
            stage2SideClearance = [stage2SideClearance, clearanceVector(2:3)]; %#ok<AGROW>
            stage2LateralError = [stage2LateralError, abs(clearGeom.pCylinder_B(2))]; %#ok<AGROW>
            stage2HeightError = [stage2HeightError, abs(clearGeom.pCylinder_B(3) - insertionLineHeightNumeric(clearGeom.pCylinder_B(1), scene))]; %#ok<AGROW>
            stage2AttitudeError = [stage2AttitudeError, max(abs(X(4:6) - scene.box.rpy))]; %#ok<AGROW>
            omega = rpyRateMapZYX(X(4:6)) * X(10:12);
            pCd_S = X(7:9) + cross(omega, rpy2rotmZYX(X(4:6)) * scene.objectCylinder.center_P);
            pCd_B = scene.box.R_S.' * pCd_S;
            stage2InsertionSpeed = [stage2InsertionSpeed, pCd_B(1)]; %#ok<AGROW>
        end
    end
end

[minClearance, clearanceIndex] = min(allClearance(:));
obstacleCount = numel(scene.hood.obstacles);
sampleIndex = ceil(clearanceIndex / obstacleCount);
obstacleIndex = mod(clearanceIndex - 1, obstacleCount) + 1;
forceUpperViolation = allForce - repmat(model.actuator.forceMax(:), 1, size(allForce, 2));
forceLowerViolation = repmat(model.actuator.forceMin(:), 1, size(allForce, 2)) - allForce;

report = struct();
report.samplesPerInterval = samplesPerInterval;
report.sampleCount = numel(allTime);
report.maxPathViolation = max([allC(:); 0]);
report.minClearance = minClearance;
report.minObstacleClearance = min(allObstacleClearance, [], 2).';
report.minStage1Clearance = min(allStage1Clearance(:));
report.minStage1ObstacleClearance = min(allStage1ObstacleClearance, [], 2).';
report.finalGap = evaluateCylinderBoxClearance(traj.Q(:, end), scene).distance;
report.minClearanceTime = allTime(sampleIndex);
report.minClearanceObstacleName = scene.hood.obstacles(obstacleIndex).name;
report.minLength = min(allLength(:));
report.maxLength = max(allLength(:));
report.maxAbsLd = max(abs(allLd(:)));
report.maxAbsLdd = max(abs(allLdd(:)));
report.maxAbsForce = max(abs(allForce(:)));
report.minSigmaMin = min(allSigma(:));
report.maxCondJ = max(allCond(:));
report.maxDynResidual = max(abs(allDyn(:)));
report.forceUpperViolationMax = max([forceUpperViolation(:); 0]);
report.forceLowerViolationMax = max([forceLowerViolation(:); 0]);
report.stage2MaxLateralError = max([stage2LateralError(:); 0]);
report.stage2MaxHeightError = max([stage2HeightError(:); 0]);
report.stage2MaxAttitudeError = max([stage2AttitudeError(:); 0]);
report.stage2MinInsertionSpeed = min([stage2InsertionSpeed(:); inf]);
report.stage2SideMinClearance = min(stage2SideClearance, [], 2).';
report.stage2Passed = report.stage2MaxLateralError <= 1e-5 && ...
    report.stage2MaxHeightError <= 1e-5 && report.stage2MaxAttitudeError <= 1e-5 && ...
    report.stage2MinInsertionSpeed >= -1e-7 && ...
    all(report.stage2SideMinClearance >= scene.collision.safeDistance - 1e-8);
report.insertionContinuousReport = validateInsertionPhaseContinuousClearance(scene);
report.insertionContinuousPassed = report.insertionContinuousReport.passed;
report.forcePassed = report.forceUpperViolationMax <= 1e-6 && report.forceLowerViolationMax <= 1e-6;
report.singularityPassed = report.minSigmaMin >= model.singularity.sigmaMinSafe && report.maxCondJ <= model.singularity.condWarning;
report.stage1ClearanceThreshold = scene.collision.safeDistance * ones(1, numel(scene.hood.obstacles));
report.stage1ClearanceThreshold(1) = scene.collision.stage1ConstraintDistance;
report.stage1ClearancePassed = all(report.minStage1ObstacleClearance >= report.stage1ClearanceThreshold - 1e-8);
report.pathPassed = report.maxPathViolation <= 1e-6 && report.stage1ClearancePassed;
report.engineeringTrajectoryPassed = report.pathPassed && report.forcePassed && ...
    report.singularityPassed && report.stage2Passed && report.insertionContinuousPassed;
end

function X = integrateToTau(X0, Fleft, Fmid, Fright, tau, h, model)
if tau == 0
    X = X0;
elseif tau <= 0.5
    X = rk4Integrate(X0, Fleft, Fmid, Fright, 0, tau, h, model);
else
    Xmid = rk4Integrate(X0, Fleft, Fmid, Fright, 0, 0.5, h, model);
    X = rk4Integrate(Xmid, Fleft, Fmid, Fright, 0.5, tau, h, model);
end
end

function X1 = rk4Integrate(X0, Fleft, Fmid, Fright, tau0, tau1, h, model)
dt = h * (tau1 - tau0);
f1 = slope(X0, Fleft, Fmid, Fright, tau0, model);
f2 = slope(X0 + 0.5*dt*f1, Fleft, Fmid, Fright, 0.5*(tau0+tau1), model);
f3 = slope(X0 + 0.5*dt*f2, Fleft, Fmid, Fright, 0.5*(tau0+tau1), model);
f4 = slope(X0 + dt*f3, Fleft, Fmid, Fright, tau1, model);
X1 = X0 + dt/6*(f1 + 2*f2 + 2*f3 + f4);
end

function xdot = slope(X, Fleft, Fmid, Fright, tau, model)
F = quadraticForce(Fleft, Fmid, Fright, tau);
xdot = stateDynamicsCompositeRigidBody(X, F, model);
end

function F = quadraticForce(Fleft, Fmid, Fright, tau)
L0 = 2*(tau - 0.5)*(tau - 1.0);
Lc = -4*tau*(tau - 1.0);
L1 = 2*tau*(tau - 0.5);
F = L0*Fleft + Lc*Fmid + L1*Fright;
end

function X = integrateImplicitAToTau(X0, Aleft, Amid, Aright, tau, h)
if tau == 0
    X = X0;
elseif tau <= 0.5
    X = rk4IntegrateImplicitA(X0, Aleft, Amid, Aright, 0, tau, h);
else
    Xmid = rk4IntegrateImplicitA(X0, Aleft, Amid, Aright, 0, 0.5, h);
    X = rk4IntegrateImplicitA(Xmid, Aleft, Amid, Aright, 0.5, tau, h);
end
end

function X1 = rk4IntegrateImplicitA(X0, Aleft, Amid, Aright, tau0, tau1, h)
dt = h * (tau1 - tau0);
f1 = implicitASlope(X0, Aleft, Amid, Aright, tau0);
f2 = implicitASlope(X0 + 0.5*dt*f1, Aleft, Amid, Aright, 0.5*(tau0+tau1));
f3 = implicitASlope(X0 + 0.5*dt*f2, Aleft, Amid, Aright, 0.5*(tau0+tau1));
f4 = implicitASlope(X0 + dt*f3, Aleft, Amid, Aright, tau1);
X1 = X0 + dt/6*(f1 + 2*f2 + 2*f3 + f4);
end

function slopeValue = implicitASlope(X, Aleft, Amid, Aright, tau)
A = quadraticAccel(Aleft, Amid, Aright, tau);
slopeValue = [X(7:12); A];
end

function A = quadraticAccel(Aleft, Amid, Aright, tau)
L0 = 2*(tau - 0.5)*(tau - 1.0);
Lc = -4*tau*(tau - 1.0);
L1 = 2*tau*(tau - 0.5);
A = L0*Aleft + Lc*Amid + L1*Aright;
end

function z = insertionLineHeightNumeric(x, scene)
x0 = scene.phase.pCylinderWaypoint_B(1);
x1 = scene.phase.pCylinderGoal_B(1);
z0 = scene.phase.pCylinderWaypoint_B(3);
z1 = scene.phase.pCylinderGoal_B(3);
lambda = (x - x0) / (x1 - x0);
z = z0 + lambda * (z1 - z0);
end
