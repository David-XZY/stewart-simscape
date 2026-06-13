function test_23_pose_length_full_tracking
% test_23_pose_length_full_tracking - 验证 Run04 完整轨迹硬验收
optRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(optRoot, 'integration'));

setup = prepareSimscapePoseLengthControl();
cleanup = onCleanup(@() closePreparedModel(setup.modelName));
output = sim(setup.modelName, ...
    'StopTime', num2str(setup.refs.t(end), 16), ...
    'ReturnWorkspaceOutputs', 'on');
report = evaluateSimscapePoseLengthControl(output.get('simout'), ...
    setup.refs, setup.model, setup.design, setup.config);

assert(report.passed, 'Run04 位姿外环长度输入控制未通过完整轨迹硬验收。');
assert(isfield(report, 'poseLengthCorrection'));
assert(isfield(report, 'filteredPoseError'));
assert(report.metrics.maxAbsPoseLengthCorrection <= setup.config.poseCorrectionLimit + 1e-9);
assert(report.metrics.post5sEquivalentPoseRippleRms < 2e-5, ...
    'Run04 后半段等效位姿高频纹波 RMS 必须低于 0.02 mm。');
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
