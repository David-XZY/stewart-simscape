function test_21_pose_length_control_contract
% test_21_pose_length_control_contract - 验证 Run04 位姿外环长度输入控制契约
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));

model = buildOptModelCustom();
config = makeSimscapePoseLengthConfig(model);
design = designSimscapePoseLengthController(model, config);
controller = initializeController('type', 'pose-length-cascade');

assert(controller.type == 9);
assert(config.poseFeedbackGain == 1.0);
assert(config.poseCorrectionLimit == 0.002);
assert(config.poseFeedbackFilterHz == 2.0);
assert(design.stable);
assert(design.useTimeVaryingReferenceJacobian);
assert(0 < design.poseFeedbackFilterAlpha && design.poseFeedbackFilterAlpha < 1);

poseError = [1e-3; 0; 0; 0; 0; 0];
lengthCorrection = config.poseFeedbackGain * sgpJacobian(model.qHome, model).Jq * poseError;
assert(all(isfinite(lengthCorrection)));
assert(max(abs(lengthCorrection)) < config.poseCorrectionLimit);

otherJacobian = sgpJacobian(model.qHome + [0.02; 0; 0; 0; 0; 0], model).Jq;
assert(max(abs(otherJacobian - sgpJacobian(model.qHome, model).Jq), [], 'all') > 1e-4);
end
