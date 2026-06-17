%% run_01_ihsid_trajectory - 两阶段 standard IHSID 轨迹生成主入口
% 文件用途：
%   构建并求解 6-UCU Stewart 平台携带运动圆柱体到固定长方体下方的两阶段轨迹优化。
%   默认链路使用 standard IHSID、CasADi MX、IPOPT/MA27、40x20 网格和
%   limited-memory Hessian。仅通过求解器与工程后验双重验收的轨迹才导出到 Simscape。
%
% 输入参数：
%   本脚本无外部输入。模型由 buildOptModelCustom 配置，场景由
%   buildCylinderBoxTransferScene 自动计算。
%
% 输出参数：
%   在 results/trajectory_planning 下保存 MAT、summary、console log、PNG 图和动画。
%
% 核心公式：
%   q0 -> qWaypoint -> qGoal；阶段 2 满足 p_C^B=[s;0;z_goal]、姿态等于长方体姿态。
%
% 优化链路位置：
%   本脚本是 opt_minimal 的唯一默认入口。Simscape 不进入 NLP，只消费通过验收后
%   导出的 references.r 和 references.rL。
clear; close all; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
optRoot = fullfile(projectRoot, 'opt_minimal');
controllerRoot = fullfile(projectRoot, 'controller');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'ihsid'));
addpath(fullfile(optRoot, 'validation'));
addpath(fullfile(optRoot, 'tools'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
scene.phase.numIntervalsApproach = 40;
scene.phase.numIntervalsInsertion = 20;
disc = buildTwoPhaseIHSDiscretization(scene, 40, 20);

resultDir = fullfile(projectRoot, 'results', 'trajectory_planning');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
tag = ['ihsid_40x20_limited_memory_', timestamp];
diaryFile = fullfile(resultDir, ['console_', tag, '.txt']);
diary(diaryFile);
diaryCleanup = onCleanup(@() diary('off'));

fprintf('\n===== standard IHSID 40x20 limited-memory 轨迹生成主入口 =====\n');
fprintf('入口：run_01_ihsid_trajectory.m\n');
fprintf('qWaypoint = [% .8f % .8f % .8f % .8f % .8f % .8f]^T\n', scene.qWaypoint);
fprintf('qGoal     = [% .8f % .8f % .8f % .8f % .8f % .8f]^T\n', scene.qGoal);
fprintf('safeDistance=%.6f m, finalGap=%.6f m\n', ...
    scene.collision.safeDistance, scene.collision.finalGap);
fprintf('box rpy(deg)=[%.3f %.3f %.3f], hood obstacles=%s\n', ...
    rad2deg(scene.box.rpy), strjoin({scene.hood.obstacles.name}, ','));
fprintf('objective weights W1: nominalStage1=%.2f, forceRate=%.2f, legAccel=%.2f, singularity=%.2f\n', ...
    model.objective.weightNominalStage1, model.objective.weightForceRate, ...
    model.objective.weightLegAccel, model.objective.weightSingularity);
fprintf('objective scales: L_dev=%.6f m, theta_dev=%.6f rad, forceRateScale=%.6f N/s, legAccelScale=%.6f m/s^2\n', ...
    model.objective.positionDeviationScale, model.objective.attitudeDeviationScale, ...
    model.objective.forceRateScale, model.objective.legAccelScale);
fprintf('T1=%.3f s, T2=%.3f s, N1=%d, N2=%d, h=%.6f s\n', ...
    disc.durationApproach, disc.durationInsertion, disc.numIntervalsApproach, ...
    disc.numIntervalsInsertion, disc.h);

ma27Info = setupCasadiIpoptMa27(projectRoot);

expectedSizes = computeExpectedSizes(disc);
fprintf('目标规模：numIntervals=%d, numel(z)=%d, numel(gEq)=%d, numel(cIneq)=%d\n', ...
    disc.numIntervals, expectedSizes.numZ, expectedSizes.numEq, expectedSizes.numIneq);
fprintf('collision certificates=%d, separator variables=%d\n', ...
    disc.numCollisionCertificates, 8*disc.numCollisionCertificates);

[z0, initialGuess] = buildInitialGuessTwoPhaseIHSImplicit(model, scene, disc);
if numel(z0) ~= expectedSizes.numZ
    error('run_01_ihsid_trajectory:InvalidInitialLength', ...
        '两阶段 IHSID 初值长度必须为 %d，当前为 %d。', expectedSizes.numZ, numel(z0));
end

solverOptions = struct('solverBackend', 'ipopt', 'hessianApproximation', 'limited-memory');
fprintf('\n===== 构建 CasADi MX standard IHSID NLP =====\n');
buildTimer = tic;
nlpData = buildCasadiImplicitIHSNLP(model, scene, disc, initialGuess, solverOptions);
buildTime = toc(buildTimer);
fprintf('NLP 构建完成：numel(z)=%d, numel(gEq)=%d, numel(cIneq)=%d, buildTime=%.3f s\n', ...
    nlpData.sizes.numZ, nlpData.sizes.numEq, nlpData.sizes.numIneq, buildTime);

tic;
[J0, gEq0, cIneq0] = nlpData.eval(z0);
tEval0 = toc;
J0 = full(J0);
gEq0 = full(gEq0);
cIneq0 = full(cIneq0);
initialData = evaluateImplicitIHSTrajectoryNumeric(z0, model, scene, disc);
initialTraj = rebuildComparisonTrajectory(initialData, scene, disc);
initialReport = validateTrajectoryDenseIHSID(initialTraj, initialData, model, scene, disc);
initialObjectiveBreakdown = computeObjectiveBreakdownImplicit(initialTraj, model, scene, disc, initialGuess);
initialFile = fullfile(resultDir, ['initial_', tag, '.mat']);
save(initialFile, 'model', 'scene', 'disc', 'z0', 'initialGuess', 'initialTraj', ...
    'initialReport', 'initialObjectiveBreakdown', 'J0', 'gEq0', 'cIneq0', 'ma27Info', 'buildTime');

fprintf('\n===== 初值诊断 =====\n');
fprintf('J0=%.8e, evalTime=%.3f s, maxEq=%.3e, maxIneqViolation=%.3e\n', ...
    J0, tEval0, max(abs(gEq0)), max([cIneq0(:); 0]));
printTrajectorySummary(initialTraj, initialReport, cIneq0, gEq0, model, scene);
printObjectiveBreakdown(initialObjectiveBreakdown, '初值目标函数分解');

fprintf('\n===== 开始 IPOPT/MA27 求解 =====\n');
solveTimer = tic;
sol = nlpData.solver('x0', z0, 'lbx', nlpData.lbz, 'ubx', nlpData.ubz, ...
    'lbg', nlpData.lbg, 'ubg', nlpData.ubg);
solveTime = toc(solveTimer);
stats = nlpData.solver.stats();
zOpt = full(sol.x);
fval = full(sol.f);
gOpt = full(sol.g);
gEqOpt = gOpt(1:nlpData.sizes.numEq);
cOpt = gOpt(nlpData.sizes.numEq+1:end);

data = evaluateImplicitIHSTrajectoryNumeric(zOpt, model, scene, disc);
traj = rebuildComparisonTrajectory(data, scene, disc);
denseReport = validateTrajectoryDenseIHSID(traj, data, model, scene, disc);
objectiveBreakdown = computeObjectiveBreakdownImplicit(data, model, scene, disc, initialGuess);
solverResult = buildSolverResult(stats, solveTime, fval, gEqOpt, cOpt, ma27Info);
result = analyzeImplicitResult(traj, denseReport, cOpt, gEqOpt, solverResult, objectiveBreakdown);

fprintf('\n===== IPOPT/MA27 求解摘要 =====\n');
fprintf('return_status=%s, iterations=%d, solveTime=%.3f s, objective=%.8e\n', ...
    solverResult.return_status, solverResult.iterations, solveTime, fval);
fprintf('max equality residual=%.3e, max inequality violation=%.3e\n', ...
    max(abs(gEqOpt)), max([cOpt(:); 0]));
printTrajectorySummary(traj, denseReport, cOpt, gEqOpt, model, scene);
printObjectiveBreakdown(objectiveBreakdown, '优化后目标函数分解');

refs = [];
references = [];
publishPassed = solverResult.success && result.engineeringTrajectoryPassed;
if publishPassed
    [refs, references] = exportTrajectoryToSimscape(traj, scene, disc);
    fprintf('Simscape 参考轨迹已发布：references.r / references.rL\n');
else
    warning('run_01_ihsid_trajectory:PublishRejected', ...
        '轨迹未同时通过求解器与工程后验，不发布 Simscape 参考轨迹。');
end
resultFile = fullfile(resultDir, ['result_', tag, '.mat']);
save(resultFile, 'model', 'scene', 'disc', 'traj', 'denseReport', 'solverResult', ...
    'refs', 'references', 'publishPassed', 'result', 'objectiveBreakdown', ...
    'initialReport', 'initialObjectiveBreakdown', ...
    'J0', 'tEval0', 'buildTime', 'ma27Info', ...
    'zOpt', 'gEqOpt', 'cOpt');

plotFiles = plotOptResult(traj, model, scene, result, resultDir, tag);
animationFile = animateStewartTrajectory(traj, model, scene, resultDir, tag);
logFile = fullfile(resultDir, ['summary_', tag, '.txt']);
writeSummaryLog(logFile, resultFile, plotFiles, animationFile, result, denseReport, solverResult, ...
    initialReport, J0, tEval0, ma27Info, nlpData.sizes, objectiveBreakdown, ...
    initialObjectiveBreakdown, scene, disc, model);

fprintf('结果 MAT 已保存：%s\n', resultFile);
fprintf('summary 已保存：%s\n', logFile);
fprintf('MATLAB console log 已保存：%s\n', diaryFile);

function sizes = computeExpectedSizes(disc)
% computeExpectedSizes - 根据两阶段区间数计算隐式 HS 变量和约束规模
sizes = struct();
sizes.numZ = 12*(disc.numNodes - 2) + 12*disc.numMidpoints + ...
    6*disc.numNodes + 6*disc.numMidpoints + ...
    6*disc.numNodes + 6*disc.numMidpoints + 8*disc.numCollisionCertificates;
sizes.numEq = 24*disc.numIntervals + 6*(disc.numNodes + disc.numMidpoints) + 6 + ...
    5*(disc.numIntervalsInsertion + 1 + disc.numIntervalsInsertion) + disc.numCollisionCertificates;
sizes.numIneq = 36*(disc.numNodes + disc.numMidpoints) + ...
    (disc.numIntervalsInsertion + 1 + disc.numIntervalsInsertion) + 10*disc.numCollisionCertificates;
end

function solverResult = buildSolverResult(stats, solveTime, fval, gEq, cIneq, ma27Info)
% buildSolverResult - 统一保存 IPOPT 统计和约束残差
solverResult = struct();
solverResult.return_status = char(stats.return_status);
solverResult.success = isfield(stats, 'success') && stats.success;
solverResult.iterations = stats.iter_count;
solverResult.solveTime = solveTime;
solverResult.fval = fval;
solverResult.maxEqResidual = max(abs(gEq(:)));
solverResult.maxIneqViolation = max([cIneq(:); 0]);
solverResult.ma27Info = ma27Info;
solverResult.stats = stats;
end

function result = analyzeImplicitResult(traj, denseReport, c, ceq, solverResult, objectiveBreakdown)
% analyzeImplicitResult - 汇总 NLP 与求解后路径检查通过标识
result = struct();
result.err.startStateNorm = norm(traj.Xnode(:, 1) - traj.xStart);
result.err.waypointPositionNorm = norm(traj.Xnode(1:6, denseReport.disc.waypointNodeIndex) - denseReport.scene.qWaypoint);
result.err.endStateNorm = norm(traj.Xnode(:, end) - traj.xEnd);
result.err.hsEqualityMax = max(abs(ceq(:)));
result.constraint.maxPathViolation = max([c(:); 0]);
result.constraint.denseMaxPathViolation = denseReport.maxPathViolation;
result.constraint.minClearance = denseReport.minClearance;
result.constraint.minObstacleClearance = denseReport.minObstacleClearance;
result.constraint.minClearanceObstacleName = denseReport.minClearanceObstacleName;
result.constraint.finalGap = denseReport.finalGap;
result.constraint.stage2SideMinClearance = denseReport.stage2SideMinClearance;
result.constraint.minSigmaMin = denseReport.minSigmaMin;
result.constraint.maxCondJ = denseReport.maxCondJ;
result.constraint.forceUpperViolationMax = denseReport.forceUpperViolationMax;
result.constraint.forceLowerViolationMax = denseReport.forceLowerViolationMax;
result.constraint.minLength = denseReport.minLength;
result.constraint.maxLength = denseReport.maxLength;
result.constraint.maxAbsLd = denseReport.maxAbsLd;
result.constraint.maxAbsLdd = denseReport.maxAbsLdd;
result.nlpPassed = solverResult.success && ...
    solverResult.maxEqResidual <= 1e-6 && solverResult.maxIneqViolation <= 1e-6;
result.postCheck.pathPassed = denseReport.pathPassed;
result.postCheck.forcePassed = denseReport.forcePassed;
result.postCheck.singularityPassed = denseReport.singularityPassed;
result.postCheck.kinematicsPassed = denseReport.kinematicsPassed;
result.postCheck.stateDynamicsPassed = denseReport.stateDynamicsPassed;
result.postCheck.geometricDynamicsPassed = denseReport.geometricDynamicsPassed;
result.postCheck.stage2Passed = denseReport.stage2Passed;
result.solverTrajectoryPassed = result.nlpPassed && denseReport.solverTrajectoryPassed;
result.engineeringTrajectoryPassed = result.nlpPassed && denseReport.engineeringTrajectoryPassed;
result.objectiveBreakdown = objectiveBreakdown;
result.solver = solverResult;
end

function printTrajectorySummary(traj, denseReport, c, ceq, model, scene)
% printTrajectorySummary - 输出圆柱体两阶段轨迹关键约束摘要
fprintf('maxEq=%.3e, maxIneqViolation=%.3e\n', max(abs(ceq(:))), max([c(:); 0]));
fprintf('minClearance=%.6f m, finalGap=%.6f m, safeDistance=%.6f m\n', ...
    denseReport.minClearance, denseReport.finalGap, scene.collision.safeDistance);
fprintf('obstacle minClearance roof/left/right=[%.6f %.6f %.6f] m, active=%s at t=%.6f\n', ...
    denseReport.minObstacleClearance, denseReport.minClearanceObstacleName, denseReport.minClearanceTime);
fprintf('stage1 minClearance=%.6f m, insertion target gap=%.6f m\n', ...
    denseReport.minStage1Clearance, scene.collision.finalGap);
fprintf('stage2 side minClearance left/right=[%.6f %.6f] m\n', ...
    denseReport.stage2SideMinClearance);
fprintf('stage2 max |y_B|=%.3e, max |z_B-zGoal|=%.3e, max |rpy-rpyBox|=%.3e, min xdot_B=%.3e\n', ...
    denseReport.stage2MaxLateralError, denseReport.stage2MaxHeightError, ...
    denseReport.stage2MaxAttitudeError, denseReport.stage2MinInsertionSpeed);
fprintf('stage2 continuous gap min=%.6f m, fixed-normal gap error=%.3e\n', ...
    denseReport.insertionContinuousReport.sampleMinGap, ...
    denseReport.insertionContinuousReport.sampleMaxGapError);
fprintf('leg length=[%.6f, %.6f], max|Ld|=%.6f, max|Ldd|=%.6f\n', ...
    denseReport.minLength, denseReport.maxLength, denseReport.maxAbsLd, denseReport.maxAbsLdd);
fprintf('max|F|=%.6f N, forcePassed=%d, force violation upper/lower=[%.3e, %.3e]\n', ...
    denseReport.maxAbsForce, denseReport.forcePassed, ...
    denseReport.forceUpperViolationMax, denseReport.forceLowerViolationMax);
fprintf('sigmaMin min=%.6f (safe %.6f), condJ max=%.6f (warning %.6f)\n', ...
    denseReport.minSigmaMin, model.singularity.sigmaMinSafe, ...
    denseReport.maxCondJ, model.singularity.condWarning);
fprintf('waypoint q error=%.3e, goal q error=%.3e\n', ...
    norm(traj.Q(:, denseReport.disc.waypointNodeIndex) - scene.qWaypoint), ...
    norm(traj.Q(:, end) - scene.qGoal));
end

function printObjectiveBreakdown(breakdown, titleText)
% printObjectiveBreakdown - 打印新四项目标函数加权值与占比
fprintf('\n===== %s =====\n', titleText);
fprintf('objectiveTotal=%.8e\n', breakdown.total);
fprintf('nominalStage1=%.8e (%.2f%%)\n', ...
    breakdown.nominalStage1, breakdown.percent.nominalStage1);
fprintf('forceRate=%.8e (%.2f%%)\n', ...
    breakdown.forceRate, breakdown.percent.forceRate);
fprintf('legAccel=%.8e (%.2f%%)\n', ...
    breakdown.legAccel, breakdown.percent.legAccel);
fprintf('singularity=%.8e (%.2f%%)\n', ...
    breakdown.singularity, breakdown.percent.singularity);
end

function writeSummaryLog(logFile, resultFile, plotFiles, animationFile, result, denseReport, solverResult, ...
    initialReport, J0, tEval0, ma27Info, sizes, objectiveBreakdown, initialObjectiveBreakdown, scene, disc, model)
% writeSummaryLog - 写出默认入口摘要文本
fid = fopen(logFile, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'standard IHSID 40x20 limited-memory summary\n');
fprintf(fid, 'resultFile: %s\n', resultFile);
fprintf(fid, 'return_status: %s\n', solverResult.return_status);
fprintf(fid, 'success: %d\n', solverResult.success);
fprintf(fid, 'iterations: %d\n', solverResult.iterations);
fprintf(fid, 'solveTime: %.6f\n', solverResult.solveTime);
fprintf(fid, 'objective: %.12e\n', solverResult.fval);
fprintf(fid, 'objectiveTotalBreakdown: %.12e\n', objectiveBreakdown.total);
fprintf(fid, 'objectiveNominalStage1: %.12e\n', objectiveBreakdown.nominalStage1);
fprintf(fid, 'objectiveForceRate: %.12e\n', objectiveBreakdown.forceRate);
fprintf(fid, 'objectiveLegAccel: %.12e\n', objectiveBreakdown.legAccel);
fprintf(fid, 'objectiveSingularity: %.12e\n', objectiveBreakdown.singularity);
fprintf(fid, 'objectiveNominalStage1Percent: %.6f\n', objectiveBreakdown.percent.nominalStage1);
fprintf(fid, 'objectiveForceRatePercent: %.6f\n', objectiveBreakdown.percent.forceRate);
fprintf(fid, 'objectiveLegAccelPercent: %.6f\n', objectiveBreakdown.percent.legAccel);
fprintf(fid, 'objectiveSingularityPercent: %.6f\n', objectiveBreakdown.percent.singularity);
fprintf(fid, 'initialObjectiveNominalStage1: %.12e\n', initialObjectiveBreakdown.nominalStage1);
fprintf(fid, 'weightNominalStage1: %.6f\n', model.objective.weightNominalStage1);
fprintf(fid, 'weightForceRate: %.6f\n', model.objective.weightForceRate);
fprintf(fid, 'weightLegAccel: %.6f\n', model.objective.weightLegAccel);
fprintf(fid, 'weightSingularity: %.6f\n', model.objective.weightSingularity);
fprintf(fid, 'positionDeviationScale: %.12e\n', model.objective.positionDeviationScale);
fprintf(fid, 'attitudeDeviationScale: %.12e\n', model.objective.attitudeDeviationScale);
fprintf(fid, 'forceRateScale: %.12e\n', model.objective.forceRateScale);
fprintf(fid, 'safeDistance: %.12e\n', scene.collision.safeDistance);
fprintf(fid, 'finalGapTarget: %.12e\n', scene.collision.finalGap);
fprintf(fid, 'stage1ConstraintDistance: %.12e\n', scene.collision.stage1ConstraintDistance);
fprintf(fid, 'numCollisionCertificates: %d\n', disc.numCollisionCertificates);
fprintf(fid, 'separatorVariables: %d\n', 8*disc.numCollisionCertificates);
fprintf(fid, 'maxEqResidual: %.12e\n', solverResult.maxEqResidual);
fprintf(fid, 'maxIneqViolation: %.12e\n', solverResult.maxIneqViolation);
fprintf(fid, 'initialJ: %.12e\n', J0);
fprintf(fid, 'initialEvalTime: %.6f\n', tEval0);
fprintf(fid, 'initialMinClearance: %.12e\n', initialReport.minClearance);
fprintf(fid, 'numZ: %d\nnumEq: %d\nnumIneq: %d\n', sizes.numZ, sizes.numEq, sizes.numIneq);
fprintf(fid, 'T1: %.6f\nT2: %.6f\nN1: %d\nN2: %d\n', ...
    disc.durationApproach, disc.durationInsertion, disc.numIntervalsApproach, disc.numIntervalsInsertion);
fprintf(fid, 'qWaypoint: %s\n', mat2str(scene.qWaypoint, 10));
fprintf(fid, 'qGoal: %s\n', mat2str(scene.qGoal, 10));
fprintf(fid, 'minClearance: %.12e\n', denseReport.minClearance);
fprintf(fid, 'minObstacleClearanceRoofLeftRight: %s\n', mat2str(denseReport.minObstacleClearance, 12));
fprintf(fid, 'minClearanceObstacleName: %s\n', denseReport.minClearanceObstacleName);
fprintf(fid, 'minStage1Clearance: %.12e\n', denseReport.minStage1Clearance);
fprintf(fid, 'minStage1ObstacleClearanceRoofLeftRight: %s\n', mat2str(denseReport.minStage1ObstacleClearance, 12));
fprintf(fid, 'finalGap: %.12e\n', denseReport.finalGap);
fprintf(fid, 'stage2SideMinClearanceLeftRight: %s\n', mat2str(denseReport.stage2SideMinClearance, 12));
fprintf(fid, 'stage2MaxLateralError: %.12e\n', denseReport.stage2MaxLateralError);
fprintf(fid, 'stage2MaxHeightError: %.12e\n', denseReport.stage2MaxHeightError);
fprintf(fid, 'stage2MaxAttitudeError: %.12e\n', denseReport.stage2MaxAttitudeError);
fprintf(fid, 'stage2MinInsertionSpeed: %.12e\n', denseReport.stage2MinInsertionSpeed);
fprintf(fid, 'stage2ContinuousMinGap: %.12e\n', denseReport.insertionContinuousReport.sampleMinGap);
fprintf(fid, 'stage2ContinuousGapError: %.12e\n', denseReport.insertionContinuousReport.sampleMaxGapError);
fprintf(fid, 'minSigmaMin: %.12e\nmaxCondJ: %.12e\n', denseReport.minSigmaMin, denseReport.maxCondJ);
fprintf(fid, 'minLength: %.12e\nmaxLength: %.12e\nmaxAbsLd: %.12e\nmaxAbsLdd: %.12e\n', ...
    denseReport.minLength, denseReport.maxLength, denseReport.maxAbsLd, denseReport.maxAbsLdd);
fprintf(fid, 'maxAbsForce: %.12e\nforcePassed: %d\n', denseReport.maxAbsForce, denseReport.forcePassed);
fprintf(fid, 'ma27ReturnStatus: %s\n', ma27Info.returnStatus);
fprintf(fid, 'ma27PrecheckTime: %.6f\n', ma27Info.elapsed);
for i = 1:numel(plotFiles)
    fprintf(fid, 'plot%d: %s\n', i, plotFiles{i});
end
fprintf(fid, 'animation: %s\n', animationFile);
fprintf(fid, 'engineeringTrajectoryPassed: %d\n', result.engineeringTrajectoryPassed);
fprintf(fid, 'simscapePublishPassed: %d\n', solverResult.success && result.engineeringTrajectoryPassed);
end
