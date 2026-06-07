function test_06_simscape_length_control_smoke
% test_06_simscape_length_control_smoke - 短时验证参数化模型、整定和结果解析
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
simscapeData = buildSimscapeLengthControlData(model, scene);
homeLength = sgpIK(model.qHome, model).L;

refs = struct();
refs.t = [0, 0.02];
refs.q = repmat(model.qHome, 1, 2);
refs.q0 = model.qHome;
refs.L = repmat(homeLength, 1, 2);
refs.Fleg = zeros(6, 2);
references = struct();
references.r = timeseries(zeros(2, 6), refs.t(:));
references.rL = timeseries(zeros(2, 6), refs.t(:));

assignin('base', 'stewart', simscapeData.stewart);
assignin('base', 'payload', simscapeData.payload);
assignin('base', 'ground', simscapeData.ground);
assignin('base', 'disturbances', simscapeData.disturbances);
assignin('base', 'references', references);
assignin('base', 'controller', initializeController('type', 'open-loop'));
assignin('base', 'Kl', ss(zeros(6)));

modelFile = fullfile(projectRoot, 'matlab', 'stewart_platform_model.slx');
load_system(modelFile);
cleanup = onCleanup(@() close_system('stewart_platform_model', 0));
configureSimscapeGravity('stewart_platform_model', simscapeData.gravity, 'enabled', false);
design = designSimscapeLengthController('stewart_platform_model', 0.5);
assignin('base', 'Kl', design.Kl);
assignin('base', 'controller', simscapeData.controller);
configureSimscapeGravity('stewart_platform_model', simscapeData.gravity);
simulationOutput = sim('stewart_platform_model', 'StopTime', '0.02', ...
    'ReturnWorkspaceOutputs', 'on');
report = evaluateSimscapeLengthControl(simulationOutput.get('simout'), refs, model, design);

assert(design.stable);
assert(design.lowFrequencyRank == 6);
assert(report.acceptance.finitePassed);
assert(~isempty(report.controlForce));
end
