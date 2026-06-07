function denseReport = validateTrajectoryDenseImplicit(traj, model, scene, disc)
% validateTrajectoryDenseImplicit - 隐式 HS 结果的密集后验验证
%
% 用途：
%   在节点、中点和每区间 20 个子采样点上验证连续轨迹。路径约束按实际几何
%   运动轨迹 X_geom=[q_poly;qdot_poly]、A_geom=qdd_poly 计算；动力学可实现性同时
%   按状态轨迹 X_state=[q_poly;v_state]、A_state=vdot_poly 和几何轨迹两种口径报告。
%
% 输入：
%   traj  struct - 隐式 HS 轨迹，含 Xnode、Anode、Amid、Unode、Umid
%   model struct - Stewart 模型、约束、奇异性阈值
%   scene struct - 运动圆柱体与固定长方体两阶段场景
%   disc  struct - HS 离散配置
%
% 输出：
%   denseReport struct - 路径、奇异性、动力学、运动学一致性和综合通过标识
%
% 核心公式：
%   X(tau)=h00*X0+h*h10*f0+h01*X1+h*h11*f1；
%   r_kin=qdot_poly-v_state，r_acc=qdd_poly-vdot_poly；
%   r_dyn,state=Jv(q)'F-Wreq(q,v_state,vdot_poly)；
%   r_dyn,geom =Jv(q)'F-Wreq(q,qdot_poly,qdd_poly)。
%
% 优化链路位置：
%   IPOPT 求解后用于后验判定；本函数不改变 NLP，只报告连续轨迹质量。

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
allDynState = [];
allDynGeom = [];
allKinResidual = [];
allAccelResidual = [];
allTime = [];
stage2LateralError = [];
stage2HeightError = [];
stage2AttitudeError = [];
stage2InsertionSpeed = [];

endpointVdotResidual = [];
midpointStateResidual = [];
midpointAccelStateResidual = [];

for intervalIndex = 1:disc.numIntervals
    X0 = traj.Xnode(:, intervalIndex);
    X1 = traj.Xnode(:, intervalIndex + 1);
    f0 = [X0(7:12); traj.Anode(:, intervalIndex)];
    f1 = [X1(7:12); traj.Anode(:, intervalIndex + 1)];
    F0 = traj.Unode(:, intervalIndex);
    Fc = traj.Umid(:, intervalIndex);
    F1 = traj.Unode(:, intervalIndex + 1);

    [~, XdotLeft, ~] = hermiteStateDerivatives(X0, X1, f0, f1, disc.h, 0);
    [~, XdotRight, ~] = hermiteStateDerivatives(X0, X1, f0, f1, disc.h, 1);
    endpointVdotResidual = [endpointVdotResidual; XdotLeft(7:12)-traj.Anode(:, intervalIndex); ...
        XdotRight(7:12)-traj.Anode(:, intervalIndex+1)]; %#ok<AGROW>

    [XmidPoly, XdotMid, ~] = hermiteStateDerivatives(X0, X1, f0, f1, disc.h, 0.5);
    midpointStateResidual = [midpointStateResidual; XmidPoly - traj.Xmid(:, intervalIndex)]; %#ok<AGROW>
    midpointAccelStateResidual = [midpointAccelStateResidual; XdotMid(7:12) - traj.Amid(:, intervalIndex)]; %#ok<AGROW>

    for subIndex = 0:samplesPerInterval
        if intervalIndex > 1 && subIndex == 0
            continue;
        end
        tau = subIndex / samplesPerInterval;
        [X, Xdot, Xdd] = hermiteStateDerivatives(X0, X1, f0, f1, disc.h, tau);
        F = quadraticControl(F0, Fc, F1, tau);

        q = X(1:6);
        vState = X(7:12);
        qdotPoly = Xdot(1:6);
        vdotPoly = Xdot(7:12);
        qddPoly = Xdd(1:6);

        rKin = qdotPoly - vState;
        rAcc = qddPoly - vdotPoly;

        pointGeom = evaluatePathConstraintsAtPoint([q; qdotPoly], F, model, scene, qddPoly);
        pointState = evaluatePathConstraintsAtPoint([q; vState], F, model, scene, vdotPoly);

        allC = [allC; pointGeom.cPath]; %#ok<AGROW>
        allLength = [allLength, pointGeom.L]; %#ok<AGROW>
        allLd = [allLd, pointGeom.Ld]; %#ok<AGROW>
        allLdd = [allLdd, pointGeom.Ldd]; %#ok<AGROW>
        allForce = [allForce, F]; %#ok<AGROW>
        allSigma = [allSigma, pointGeom.sigmaMin]; %#ok<AGROW>
        allCond = [allCond, pointGeom.condJ]; %#ok<AGROW>
        clearanceVector = pointGeom.collisionDistances(:);
        allClearance = [allClearance; clearanceVector]; %#ok<AGROW>
        allObstacleClearance = [allObstacleClearance, clearanceVector]; %#ok<AGROW>
        sampleTime = disc.tNode(intervalIndex) + tau*disc.h;
        if sampleTime <= disc.durationApproach + 1e-12
            allStage1Clearance = [allStage1Clearance; clearanceVector]; %#ok<AGROW>
            allStage1ObstacleClearance = [allStage1ObstacleClearance, clearanceVector]; %#ok<AGROW>
        end
        allDynState = [allDynState, pointState.dynAux.rDyn]; %#ok<AGROW>
        allDynGeom = [allDynGeom, pointGeom.dynAux.rDyn]; %#ok<AGROW>
        allKinResidual = [allKinResidual, rKin]; %#ok<AGROW>
        allAccelResidual = [allAccelResidual, rAcc]; %#ok<AGROW>
        allTime = [allTime, disc.tNode(intervalIndex) + tau*disc.h]; %#ok<AGROW>

        if sampleTime >= disc.durationApproach - 1e-12
            clearGeom = evaluateCylinderBoxClearance(q, scene);
            stage2SideClearance = [stage2SideClearance, clearanceVector(2:3)]; %#ok<AGROW>
            stage2LateralError = [stage2LateralError, abs(clearGeom.pCylinder_B(2))]; %#ok<AGROW>
            stage2HeightError = [stage2HeightError, abs(clearGeom.pCylinder_B(3) - insertionLineHeightNumeric(clearGeom.pCylinder_B(1), scene))]; %#ok<AGROW>
            stage2AttitudeError = [stage2AttitudeError, max(abs(q(4:6) - scene.box.rpy))]; %#ok<AGROW>
            omega = rpyRateMapZYX(q(4:6)) * qdotPoly(4:6);
            pCd_S = qdotPoly(1:3) + cross(omega, rpy2rotmZYX(q(4:6)) * scene.objectCylinder.center_P);
            pCd_B = scene.box.R_S.' * pCd_S;
            stage2InsertionSpeed = [stage2InsertionSpeed, pCd_B(1)]; %#ok<AGROW>
        end
    end
end

[minSigma, sigmaIndex] = min(allSigma(:));
[maxCond, condIndex] = max(allCond(:));
[minClearance, clearanceIndex] = min(allClearance(:));
clearanceSampleIndex = ceil(clearanceIndex / numel(scene.hood.obstacles));
clearanceObstacleIndex = mod(clearanceIndex - 1, numel(scene.hood.obstacles)) + 1;
[maxKin, kinIndex] = max(abs(allKinResidual(:)));
[maxAcc, accIndex] = max(abs(allAccelResidual(:)));
[maxDynState, dynStateIndex] = max(abs(allDynState(:)));
[maxDynGeom, dynGeomIndex] = max(abs(allDynGeom(:)));

denseReport = struct();
denseReport.samplesPerInterval = samplesPerInterval;
denseReport.sampleCount = numel(allTime);
denseReport.forceInterpolation = 'quadratic Lagrange through Fnode(k), Fmid(k), Fnode(k+1)';
denseReport.pathDefinition = 'path constraints use X_geom=[q_poly;qdot_poly] and A_geom=qdd_poly';
denseReport.dynamicsDefinition = 'state dynamics use vdot_poly; geometric dynamics use qdd_poly';
denseReport.maxPathViolation = max([allC(:); 0]);
denseReport.minClearance = minClearance;
denseReport.minObstacleClearance = min(allObstacleClearance, [], 2).';
denseReport.minStage1Clearance = min(allStage1Clearance(:));
denseReport.minStage1ObstacleClearance = min(allStage1ObstacleClearance, [], 2).';
denseReport.finalGap = evaluateCylinderBoxClearance(traj.Q(:, end), scene).distance;
denseReport.minClearanceFlatIndex = clearanceIndex;
denseReport.minClearanceTime = allTime(clearanceSampleIndex);
denseReport.minClearanceObstacleIndex = clearanceObstacleIndex;
denseReport.minClearanceObstacleName = scene.hood.obstacles(clearanceObstacleIndex).name;
denseReport.minLength = min(allLength(:));
denseReport.maxLength = max(allLength(:));
denseReport.maxAbsLd = max(abs(allLd(:)));
denseReport.maxAbsLdd = max(abs(allLdd(:)));
denseReport.maxAbsForce = max(abs(allForce(:)));
denseReport.minSigmaMin = minSigma;
denseReport.minSigmaTime = allTime(sigmaIndex);
denseReport.maxCondJ = maxCond;
denseReport.maxCondTime = allTime(condIndex);
denseReport.maxKinematicResidual = maxKin;
denseReport.maxKinematicResidualTime = allTime(ceil(kinIndex/6));
denseReport.maxAccelConsistencyResidual = maxAcc;
denseReport.maxAccelConsistencyResidualTime = allTime(ceil(accIndex/6));
denseReport.maxDynResidualState = maxDynState;
denseReport.maxDynResidualStateTime = allTime(ceil(dynStateIndex/6));
denseReport.maxDynResidualGeometric = maxDynGeom;
denseReport.maxDynResidualGeometricTime = allTime(ceil(dynGeomIndex/6));
denseReport.maxDynResidual = denseReport.maxDynResidualGeometric;
denseReport.maxEndpointVdotResidual = max(abs(endpointVdotResidual(:)));
denseReport.maxMidpointStateResidual = max(abs(midpointStateResidual(:)));
denseReport.maxMidpointAccelStateResidual = max(abs(midpointAccelStateResidual(:)));
denseReport.stage2MaxLateralError = max([stage2LateralError(:); 0]);
denseReport.stage2MaxHeightError = max([stage2HeightError(:); 0]);
denseReport.stage2MaxAttitudeError = max([stage2AttitudeError(:); 0]);
denseReport.stage2MinInsertionSpeed = min([stage2InsertionSpeed(:); inf]);
denseReport.stage2SideMinClearance = min(stage2SideClearance, [], 2).';
denseReport.stage2Passed = denseReport.stage2MaxLateralError <= 1e-5 && ...
    denseReport.stage2MaxHeightError <= 1e-5 && ...
    denseReport.stage2MaxAttitudeError <= 1e-5 && ...
    denseReport.stage2MinInsertionSpeed >= -1e-7 && ...
    all(denseReport.stage2SideMinClearance >= scene.collision.safeDistance - 1e-8);
denseReport.insertionContinuousReport = validateInsertionPhaseContinuousClearance(scene);
denseReport.insertionContinuousPassed = denseReport.insertionContinuousReport.passed;
forceUpperViolation = allForce - repmat(model.actuator.forceMax(:), 1, size(allForce, 2));
forceLowerViolation = repmat(model.actuator.forceMin(:), 1, size(allForce, 2)) - allForce;
denseReport.forceUpperViolationMax = max([forceUpperViolation(:); 0]);
denseReport.forceLowerViolationMax = max([forceLowerViolation(:); 0]);
denseReport.stage1ClearanceThreshold = scene.collision.safeDistance * ones(1, numel(scene.hood.obstacles));
denseReport.stage1ClearanceThreshold(1) = scene.collision.stage1ConstraintDistance;
denseReport.stage1ClearancePassed = all(denseReport.minStage1ObstacleClearance >= denseReport.stage1ClearanceThreshold - 1e-8);
denseReport.pathPassed = denseReport.maxPathViolation <= 1e-6 && denseReport.stage1ClearancePassed;
denseReport.forcePassed = denseReport.forceUpperViolationMax <= 1e-6 && ...
    denseReport.forceLowerViolationMax <= 1e-6;
denseReport.singularityPassed = denseReport.minSigmaMin >= model.singularity.sigmaMinSafe && ...
    denseReport.maxCondJ <= model.singularity.condWarning;
denseReport.kinematicsPassed = denseReport.maxKinematicResidual <= 1e-5 && ...
    denseReport.maxAccelConsistencyResidual <= 1e-5;
denseReport.stateDynamicsPassed = denseReport.maxDynResidualState <= 1e-5;
denseReport.geometricDynamicsPassed = denseReport.maxDynResidualGeometric <= 1e-5;
denseReport.solverTrajectoryPassed = denseReport.pathPassed && denseReport.forcePassed && ...
    denseReport.singularityPassed && denseReport.stateDynamicsPassed && ...
    denseReport.stage2Passed && denseReport.insertionContinuousPassed;
denseReport.engineeringTrajectoryPassed = denseReport.pathPassed && denseReport.forcePassed && ...
    denseReport.singularityPassed && denseReport.kinematicsPassed && ...
    denseReport.geometricDynamicsPassed && denseReport.stage2Passed && denseReport.insertionContinuousPassed;
denseReport.disc = disc;
denseReport.scene = scene;
end

function [X, Xdot, Xdd] = hermiteStateDerivatives(X0, X1, f0, f1, h, tau)
h00 = 2*tau^3 - 3*tau^2 + 1;
h10 = tau^3 - 2*tau^2 + tau;
h01 = -2*tau^3 + 3*tau^2;
h11 = tau^3 - tau^2;
X = h00*X0 + h*h10*f0 + h01*X1 + h*h11*f1;

h00d = 6*tau^2 - 6*tau;
h10d = 3*tau^2 - 4*tau + 1;
h01d = -6*tau^2 + 6*tau;
h11d = 3*tau^2 - 2*tau;
Xdot = (h00d/h)*X0 + h10d*f0 + (h01d/h)*X1 + h11d*f1;

h00dd = 12*tau - 6;
h10dd = 6*tau - 4;
h01dd = -12*tau + 6;
h11dd = 6*tau - 2;
Xdd = (h00dd/h^2)*X0 + (h10dd/h)*f0 + (h01dd/h^2)*X1 + (h11dd/h)*f1;
end

function F = quadraticControl(F0, Fc, F1, tau)
L0 = 2*(tau - 0.5)*(tau - 1.0);
Lc = -4*tau*(tau - 1.0);
L1 = 2*tau*(tau - 0.5);
F = L0*F0 + Lc*Fc + L1*F1;
end

function z = insertionLineHeightNumeric(x, scene)
x0 = scene.phase.pCylinderWaypoint_B(1);
x1 = scene.phase.pCylinderGoal_B(1);
z0 = scene.phase.pCylinderWaypoint_B(3);
z1 = scene.phase.pCylinderGoal_B(3);
lambda = (x - x0) / (x1 - x0);
z = z0 + lambda * (z1 - z0);
end
