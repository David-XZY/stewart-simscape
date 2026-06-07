function test_01_ihsid_contract
% test_01_ihsid_contract - 验证默认网格、IHSID 打包解包和 NLP 尺寸
[projectRoot, optRoot] = addActivePaths();
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
defaultDisc = buildTwoPhaseIHSDiscretization(scene);
assert(defaultDisc.numIntervalsApproach == 40);
assert(defaultDisc.numIntervalsInsertion == 20);

disc = buildTwoPhaseIHSDiscretization(scene, 2, 1);
[z0, initialData] = buildInitialGuessTwoPhaseIHSImplicit(model, scene, disc);
[Xnode, Xmid, Anode, Amid, Fnode, Fmid, Xinternal, separator] = ...
    unpackIHSDecisionImplicit(z0, scene, disc);
zRoundTrip = packIHSDecisionImplicit(Xinternal, Xmid, Anode, Amid, Fnode, Fmid, disc, separator);
assert(isequal(z0, zRoundTrip), 'IHSID 打包/解包往返不一致。');
assert(size(Xnode, 2) == disc.numNodes);

solverOptions = struct('maxIter', 1, 'printLevel', 0, 'hessianApproximation', 'limited-memory');
nlpData = buildCasadiImplicitIHSNLP(model, scene, disc, initialData, solverOptions);
expectedNumZ = 12*(disc.numNodes-2) + 12*disc.numMidpoints + ...
    6*disc.numNodes + 6*disc.numMidpoints + 6*disc.numNodes + ...
    6*disc.numMidpoints + 8*disc.numCollisionCertificates;
expectedNumEq = 24*disc.numIntervals + 6*(disc.numNodes + disc.numMidpoints) + 6 + ...
    5*(disc.numIntervalsInsertion + 1 + disc.numIntervalsInsertion) + disc.numCollisionCertificates;
expectedNumIneq = 36*(disc.numNodes + disc.numMidpoints) + ...
    (disc.numIntervalsInsertion + 1 + disc.numIntervalsInsertion) + 10*disc.numCollisionCertificates;
assert(nlpData.sizes.numZ == expectedNumZ);
assert(nlpData.sizes.numEq == expectedNumEq);
assert(nlpData.sizes.numIneq == expectedNumIneq);
assert(strcmp(nlpData.solverBackend, 'ipopt'));
assert(strcmp(nlpData.opts.ipopt.linear_solver, 'ma27'));
assert(strcmp(nlpData.opts.ipopt.hessian_approximation, 'limited-memory'));
assert(isfolder(optRoot));
end

function [projectRoot, optRoot] = addActivePaths
testRoot = fileparts(mfilename('fullpath'));
optRoot = fileparts(testRoot);
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'), fullfile(optRoot, 'ihsid'), ...
    fullfile(optRoot, 'validation'), fullfile(optRoot, 'integration'));
end
