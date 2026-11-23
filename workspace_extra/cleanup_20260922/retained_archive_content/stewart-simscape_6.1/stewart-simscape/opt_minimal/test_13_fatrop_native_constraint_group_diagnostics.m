function test_13_fatrop_native_constraint_group_diagnostics()
% test_13_fatrop_native_constraint_group_diagnostics - 验证 FATROP-native 约束组诊断
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildLocalDisc(scene, 2, 1);
[z0, ~] = buildInitialGuessFatropNativeHS(model, scene, disc);

diag = diagnoseFatropNativeConstraintGroups(z0, model, scene, disc, ...
    struct('constraintProfile', 'full', 'insertionMode', 'soft'));

requiredFields = ["maxPathViolation", "maxRoofViolation", "maxLeftSideViolation", ...
    "maxRightSideViolation", "maxInsertionMonotonicViolation", "maxIneqViolation", ...
    "maxSoftInsertionResidual", "maxSoftSideCollisionResidual"];
for i = 1:numel(requiredFields)
    assert(isfield(diag, requiredFields(i)), '缺少约束组诊断字段 %s。', requiredFields(i));
end
assert(diag.maxIneqViolation == max([diag.maxPathViolation, diag.maxRoofViolation, ...
    diag.maxLeftSideViolation, diag.maxRightSideViolation, ...
    diag.maxInsertionMonotonicViolation]), ...
    '总不等式违反应等于各不等式约束组最大值。');
assert(diag.maxSoftInsertionResidual >= 0, 'soft insertion 残差应为非负诊断量。');

fprintf('TEST_13_OK fatrop native constraint group diagnostics\n');
end

function disc = buildLocalDisc(scene, n1, n2)
disc = struct();
disc.numIntervalsApproach = n1;
disc.numIntervalsInsertion = n2;
disc.numIntervals = n1 + n2;
disc.numNodes = disc.numIntervals + 1;
disc.numMidpoints = disc.numIntervals;
disc.durationApproach = scene.phase.durationApproach;
disc.durationInsertion = scene.phase.durationInsertion;
disc.duration = disc.durationApproach + disc.durationInsertion;
disc.hApproach = disc.durationApproach / n1;
disc.hInsertion = disc.durationInsertion / n2;
if abs(disc.hApproach - disc.hInsertion) > 1e-12
    error('test_13_fatrop_native_constraint_group_diagnostics:NonUniformStep', ...
        '测试网格必须保持统一步长。');
end
disc.h = disc.hApproach;
disc.tNode = linspace(0, disc.duration, disc.numNodes);
disc.tMid = disc.tNode(1:end-1) + disc.h/2;
disc.waypointNodeIndex = n1 + 1;
disc.stage1NodeIndices = 1:(n1 + 1);
disc.stage2NodeIndices = (n1 + 1):disc.numNodes;
disc.stage1MidIndices = 1:n1;
disc.stage2MidIndices = (n1 + 1):disc.numIntervals;
disc.numStage1CollisionPoints = (n1 + 1) + n1;
disc.numAllCollisionPoints = disc.numNodes + disc.numMidpoints;
disc.numCollisionCertificates = disc.numStage1CollisionPoints + 2*disc.numAllCollisionPoints;
end
