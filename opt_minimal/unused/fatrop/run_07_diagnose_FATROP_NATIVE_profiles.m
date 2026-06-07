function result = run_07_diagnose_FATROP_NATIVE_profiles(varargin)
% run_07_diagnose_FATROP_NATIVE_profiles - 小网格定位 FATROP-native 退化约束层
parser = inputParser();
parser.addParameter('grid', [4 2]);
parser.addParameter('maxIter', 20);
parser.addParameter('profiles', ["dynamics", "actuator", "actuator_collision", "no_insertion", "full"]);
parser.parse(varargin{:});

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
grid = parser.Results.grid;
disc = buildNativeDiscLocal(scene, grid(1), grid(2));
[z0, initialGuess] = buildInitialGuessFatropNativeHS(model, scene, disc);

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
resultDir = fullfile(projectRoot, 'opt_minimal', 'results', ...
    ['diagnose_FATROP_NATIVE_profiles_', timestamp]);
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

profiles = string(parser.Results.profiles);
rows = struct([]);
for profileIndex = 1:numel(profiles)
    row = runOneProfile(profiles(profileIndex), model, scene, disc, initialGuess, z0, ...
        parser.Results.maxIter, resultDir);
    rows = [rows; row]; %#ok<AGROW>
end

summaryTable = struct2table(rows);
csvFile = fullfile(resultDir, 'profile_diagnostic_summary.csv');
writetable(summaryTable, csvFile);
save(fullfile(resultDir, 'profile_diagnostic_results.mat'), ...
    'summaryTable', 'model', 'scene', 'disc');

result = struct('resultDir', resultDir, 'summaryTable', summaryTable, 'csvFile', csvFile);
disp(summaryTable);
fprintf('\nFATROP-native 约束分层诊断结果目录：%s\n', resultDir);
end

function row = runOneProfile(profile, model, scene, disc, initialGuess, z0, maxIter, resultDir)
fprintf('\n===== FATROP-native profile=%s N1=%d N2=%d =====\n', profile, ...
    disc.numIntervalsApproach, disc.numIntervalsInsertion);
row = baseRow(profile, disc);
solverOptions = struct('solverBackend', 'fatrop', 'fatropStructure', 'manual', ...
    'maxIter', maxIter, 'constraintProfile', char(profile));
solverLogFile = fullfile(resultDir, sprintf('fatrop_profile_%s.log', char(profile)));
solveTimer = tic;
diaryCleanup = []; %#ok<NASGU>
try
    nlpData = buildCasadiFatropNativeHSNLP(model, scene, disc, initialGuess, solverOptions);
    row.numVariables = nlpData.sizes.numZ;
    row.numEq = nlpData.sizes.numEq;
    row.numIneq = nlpData.sizes.numIneq;
    row.ngMax = max(nlpData.manualStructure.ng);
    row.ngSum = sum(nlpData.manualStructure.ng);
    diary(solverLogFile);
    diaryCleanup = onCleanup(@() diary('off'));
    sol = nlpData.solver('x0', z0, 'lbx', nlpData.lbz, 'ubx', nlpData.ubz, ...
        'lbg', nlpData.lbg, 'ubg', nlpData.ubg);
    row.solveTime_s = toc(solveTimer);
    stats = nlpData.solver.stats();
    clear diaryCleanup;
    gOpt = full(sol.g);
    eqMask = logical(full(nlpData.equalityMask));
    groupDiag = diagnoseFatropNativeConstraintGroups(full(sol.x), model, scene, disc, ...
        struct('constraintProfile', char(profile), ...
        'insertionMode', nlpData.nativeInsertionMode, ...
        'sideCollisionScope', nlpData.nativeSideCollisionScope, ...
        'sideCollisionMode', nlpData.nativeSideCollisionMode, ...
        'sideCollisionMask', nlpData.nativeSideCollisionMask));
    row.solverStatus = string(readStatus(stats));
    row.solverSuccess = readSuccess(stats);
    row.objective = full(sol.f);
    row.maxEqResidual = maxOrZero(abs(gOpt(eqMask)));
    row.maxIneqViolation = maxOrZero([gOpt(~eqMask); 0]);
    row.maxPathViolation = groupDiag.maxPathViolation;
    row.maxRoofViolation = groupDiag.maxRoofViolation;
    row.maxLeftSideViolation = groupDiag.maxLeftSideViolation;
    row.maxRightSideViolation = groupDiag.maxRightSideViolation;
    row.maxInsertionMonotonicViolation = groupDiag.maxInsertionMonotonicViolation;
    row.maxSoftInsertionResidual = groupDiag.maxSoftInsertionResidual;
    row.maxSoftSideCollisionResidual = groupDiag.maxSoftSideCollisionResidual;
catch ME
    clear diaryCleanup;
    row.solveTime_s = toc(solveTimer);
    row.solverStatus = "FAILED";
    row.failureReason = string(compactMessageLocal(ME.message));
end
solverLogText = readTextIfExistsLocal(solverLogFile);
row.degenerateJacobianCount = countDegenerateJacobianLocal(solverLogText);
end

function row = baseRow(profile, disc)
row = struct();
row.profile = string(profile);
row.N1 = disc.numIntervalsApproach;
row.N2 = disc.numIntervalsInsertion;
row.numVariables = NaN;
row.numEq = NaN;
row.numIneq = NaN;
row.ngMax = NaN;
row.ngSum = NaN;
row.solverStatus = "";
row.solverSuccess = false;
row.solveTime_s = NaN;
row.objective = NaN;
row.maxEqResidual = NaN;
row.maxIneqViolation = NaN;
row.maxPathViolation = NaN;
row.maxRoofViolation = NaN;
row.maxLeftSideViolation = NaN;
row.maxRightSideViolation = NaN;
row.maxInsertionMonotonicViolation = NaN;
row.maxSoftInsertionResidual = NaN;
row.maxSoftSideCollisionResidual = NaN;
row.degenerateJacobianCount = NaN;
row.failureReason = "";
end

function disc = buildNativeDiscLocal(scene, n1, n2)
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
    error('run_07_diagnose_FATROP_NATIVE_profiles:NonUniformStep', ...
        '诊断网格必须保持统一步长。');
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

function status = readStatus(stats)
status = char(string(getStatSafeLocal(stats, 'return_status', "")));
if strlength(string(status)) == 0
    status = char(string(getStatSafeLocal(stats, 'fatrop.return_flag', "")));
end
end

function success = readSuccess(stats)
success = false;
if isfield(stats, 'success')
    success = logical(stats.success);
end
end

function value = getStatSafeLocal(stats, dottedName, defaultValue)
if nargin < 3
    defaultValue = NaN;
end
parts = split(string(dottedName), '.');
value = stats;
for i = 1:numel(parts)
    key = char(parts(i));
    if isstruct(value) && isfield(value, key)
        value = value.(key);
    else
        value = defaultValue;
        return;
    end
end
end

function text = readTextIfExistsLocal(fileName)
if strlength(string(fileName)) > 0 && exist(fileName, 'file')
    text = fileread(fileName);
else
    text = '';
end
end

function n = countDegenerateJacobianLocal(text)
n = numel(regexp(text, 'degenerate Jacobian', 'match'));
end

function msg = compactMessageLocal(msg)
msg = regexprep(msg, '\s+', ' ');
msg = strtrim(msg);
end

function value = maxOrZero(v)
if isempty(v)
    value = 0;
else
    value = max(v);
end
end
