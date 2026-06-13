function test_21_pose_length_control_contract
% test_21_pose_length_control_contract - 验证 Run04 位姿外环长度输入控制契约
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

model = buildOptModelCustom();
config = makeSimscapePoseLengthConfig(model);
design = designSimscapePoseLengthController(model, config);
controller = initializeController('type', 'pose-length-cascade');

assert(controller.type == 9);
assert(config.poseFeedbackGain == 0.3);
assert(config.poseCorrectionLimit == 0.002);
assert(design.stable);
assert(isequal(size(design.poseToLengthGain), [6, 6]));
assert(rank(design.poseToLengthGain) == 6);

poseError = [1e-3; 0; 0; 0; 0; 0];
lengthCorrection = design.poseToLengthGain * poseError;
assert(all(isfinite(lengthCorrection)));
assert(max(abs(lengthCorrection)) < config.poseCorrectionLimit);
end
