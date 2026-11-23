%% test_09_fatrop_backend_contract - FATROP 后端可用性最小契约测试
% 只有 FATROP toy NLP 能实际 solve，才认为本机 FATROP 后端可用。
clear; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

assert(casadi.has_nlpsol('fatrop'), 'CasADi 必须能发现 fatrop nlpsol 插件。');
toyResult = solveFatropToyProblem();
assert(toyResult.success, 'FATROP toy problem 必须能实际 solve。失败原因：%s', toyResult.failureReason);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildLocalDisc(scene, 2, 1);
[~, ihsGuess] = buildInitialGuessTwoPhaseIHSImplicit(model, scene, disc);
[~, dmsGuess] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc);

ipoptOptions = struct('solverBackend', 'ipopt', 'maxIter', 3, 'printLevel', 0, ...
    'hessianApproximation', 'limited-memory', 'acceptableTol', 1e-3, 'acceptableIter', 1);
fatropOptions = struct('solverBackend', 'fatrop', 'fatropStructure', 'auto');

ihsIpopt = buildCasadiImplicitIHSNLP(model, scene, disc, ihsGuess, ipoptOptions);
assert(strcmp(ihsIpopt.solverBackend, 'ipopt'), 'IHSID-IPOPT 必须记录 solverBackend=ipopt。');

ihsResult = tryBuildFatrop(@() buildCasadiImplicitIHSNLP(model, scene, disc, ihsGuess, fatropOptions));
dmsResult = tryBuildFatrop(@() buildCasadiImplicitDMSNLP(model, scene, disc, dmsGuess, fatropOptions));
assert(~ihsResult.calledFatrop, 'IHSID-FATROP-auto 初始化失败时不能记为项目 FATROP 可用。');
assert(~dmsResult.calledFatrop, 'DMSID-FATROP-auto 初始化失败时不能记为项目 FATROP 可用。');
assert(~isempty(ihsResult.failureReason), 'IHSID-FATROP-auto 失败原因必须保留。');
assert(~isempty(dmsResult.failureReason), 'DMSID-FATROP-auto 失败原因必须保留。');

fprintf('TEST_09_OK\n');

function result = solveFatropToyProblem()
import casadi.*
result = struct('success', false, 'failureReason', '');
try
    x0 = MX.sym('x0');
    u0 = MX.sym('u0');
    x1 = MX.sym('x1');
    u1 = MX.sym('u1');
    z = [x0; u0; x1; u1];
    g = x1 - x0 - u0;
    nlp = struct('x', z, 'f', sumsqr(z), 'g', g);
    opts = struct();
    opts.structure_detection = 'manual';
    opts.N = 1;
    opts.nx = {1, 1};
    opts.nu = {1, 1};
    opts.ng = {0, 0};
    opts.equality = {true};
    opts.print_time = false;
    solver = nlpsol('toy_fatrop_manual', 'fatrop', nlp, opts);
    sol = solver('x0', zeros(4, 1), 'lbx', -10*ones(4, 1), 'ubx', 10*ones(4, 1), ...
        'lbg', 0, 'ubg', 0); %#ok<NASGU>
    stats = solver.stats();
    result.success = isfield(stats, 'success') && stats.success;
catch ME
    result.failureReason = regexprep(ME.message, '\s+', ' ');
end
end

function result = tryBuildFatrop(buildFcn)
result = struct('calledFatrop', false, 'failureReason', '');
try
    nlpData = buildFcn();
    result.calledFatrop = strcmp(nlpData.solverBackend, 'fatrop') && ...
        strcmp(nlpData.fatropStructure, 'auto') && ...
        strcmp(nlpData.opts.structure_detection, 'auto');
catch ME
    result.failureReason = regexprep(ME.message, '\s+', ' ');
    result.calledFatrop = false;
end
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
