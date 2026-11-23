function test_13_length_cascade_evaluation_contract
% test_13_length_cascade_evaluation_contract - 验证纯长度验收不读取力信号
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));

model = buildOptModelCustom();
config = makeSimscapeLengthCascadeConfig(model);
design = designSimscapeLengthCascadeController(config);
time = (0:config.sampleTime:0.1).';
refs = struct('t', time.', 'q', repmat(model.qHome, 1, numel(time)), ...
    'qd', zeros(6, numel(time)), 'q0', model.qHome, ...
    'Lref', repmat(sgpIK(model.qHome, model).L, 1, numel(time)), ...
    'Ldref', zeros(6, numel(time)));
zeroSignal = timeseries(zeros(numel(time), 6), time);
simout = struct('y', struct('dLm', zeroSignal), ...
    'x', struct('Xr', zeroSignal), 'uFeedback', zeroSignal);

report = evaluateSimscapeLengthCascadeControl(simout, refs, model, design, config);
assert(report.passed);
assert(~isfield(report, 'controlForce'));
assert(isfield(report, 'LdCmd'));
assert(isfield(report, 'servoCommand'));
assert(isfield(report.metrics, 'velocityErrorRms'));
assert(isfield(report.acceptance, 'accelerationPassed'));
end
