function report = validateTrajectoryDenseIHSID(traj, data, model, scene, disc)
% validateTrajectoryDenseIHSID - standard IHSID 的统一 dense 后验与工程验收
%
% IHSID 的 dense 动力学残差用于评价配点插值质量，不直接作为工程失败条件；
% 工程验收由路径、驱动力、奇异性、第二阶段插入和连续间隙共同决定。
report = validateTrajectoryDenseImplicit(traj, model, scene, disc);
report.trajectoryQualityDiagnostics = struct( ...
    'maxDenseDynResidualGeometric', report.maxDynResidualGeometric, ...
    'maxDenseKinematicResidual', report.maxKinematicResidual, ...
    'note', 'IHSID dense 动力学残差是配点插值质量诊断，不直接作为工程失败条件。');
report.engineeringTrajectoryPassed = report.pathPassed && report.forcePassed && ...
    report.singularityPassed && report.stage2Passed && report.insertionContinuousPassed;
report.maxDefectResidual = data.maxDefectResidual;
report.maxMidConsistencyResidual = data.maxMidConsistencyResidual;
report.method = 'IHSID';
end
