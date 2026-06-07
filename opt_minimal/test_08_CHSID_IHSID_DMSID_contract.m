%% test_08_CHSID_IHSID_DMSID_contract - 三种隐式动力学方法结构契约测试
% 本测试只检查 IHSID 新增中点状态变量后的规模关系和主入口默认方法，
% 不承担正式 20x10 自检求解，便于在改代码前先锁定本轮重构边界。
clear; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildLocalDisc(scene, 2, 1);

solverOptions = struct('maxIter', 3, 'printLevel', 0, ...
    'hessianApproximation', 'limited-memory', 'acceptableTol', 1e-3, 'acceptableIter', 1);
[z0CHS, initialGuess] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc); %#ok<ASGLU>

chsid = buildCasadiImplicitHSNLP(model, scene, disc, initialGuess, solverOptions);
ihsid = buildCasadiImplicitIHSNLP(model, scene, disc, initialGuess, solverOptions);
dmsid = buildCasadiImplicitDMSNLP(model, scene, disc, initialGuess, solverOptions);

assert(strcmp(chsid.method, 'CHSID'), 'CHSID builder 必须返回 CHSID。');
assert(strcmp(ihsid.method, 'IHSID'), 'IHSID builder 必须返回 IHSID。');
assert(strcmp(dmsid.method, 'DMSID'), 'DMSID builder 必须返回 DMSID。');
assert(ihsid.sizes.numZ == chsid.sizes.numZ + 12*disc.numIntervals, ...
    'IHSID 变量数必须比 CHSID 多 12*numIntervals 个中点状态。');
assert(ihsid.sizes.numEq == chsid.sizes.numEq + 12*disc.numIntervals, ...
    'IHSID 等式数必须比 CHSID 多 12*numIntervals 条中点一致性约束。');
assert(ihsid.sizes.numIneq == chsid.sizes.numIneq, ...
    'IHSID 不等式数量必须与 CHSID 一致。');
assert(strcmp(ihsid.opts.ipopt.linear_solver, chsid.opts.ipopt.linear_solver) && ...
       strcmp(ihsid.opts.ipopt.linear_solver, dmsid.opts.ipopt.linear_solver), ...
       '三种 ID 方法必须使用同一个 IPOPT linear_solver。');
assert(strcmp(ihsid.opts.ipopt.hessian_approximation, chsid.opts.ipopt.hessian_approximation) && ...
       strcmp(ihsid.opts.ipopt.hessian_approximation, dmsid.opts.ipopt.hessian_approximation), ...
       '三种 ID 方法必须使用同一个 Hessian 设置。');

run4Source = fileread(fullfile(projectRoot, 'opt_minimal', 'run_04_compare_CHSID_IHSID_DMSID.m'));
assert(contains(run4Source, "parser.addParameter('methods', {'CHSID','IHSID','DMSID'});") || ...
       contains(run4Source, "parser.addParameter('methods', {'CHSID', 'IHSID', 'DMSID'});"), ...
       'run_04 默认方法必须且只能为 CHSID、IHSID、DMSID。');
forbiddenMethods = {['CHS','ED'], ['DMS','ED'], ['HS','-E'], ['DMS','-E']};
assert(~any(cellfun(@(name) contains(run4Source, name), forbiddenMethods)), ...
       'run_04 active 入口不得保留 ED 方法名。');

fprintf('TEST_08_OK\n');

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
assert(abs(disc.hApproach - disc.hInsertion) < 1e-12, '测试网格必须保持统一步长。');
disc.h = disc.hApproach;
disc.tNode = linspace(0, disc.duration, disc.numNodes);
disc.tMid = disc.tNode(1:end-1) + disc.h/2;
disc.waypointNodeIndex = n1 + 1;
disc.stage2NodeIndices = disc.waypointNodeIndex:disc.numNodes;
disc.stage2MidIndices = (n1 + 1):disc.numIntervals;
disc.numStage1CollisionPoints = (n1 + 1) + n1;
disc.numAllCollisionPoints = disc.numNodes + disc.numMidpoints;
disc.numCollisionCertificates = disc.numStage1CollisionPoints + 2*disc.numAllCollisionPoints;
end
