function test_37_pwm_feedback_source_export_contract
% test_37_pwm_feedback_source_export_contract - 验证反馈源消融实验导出契约
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'pwm_identification'), fullfile(controllerRoot, 'ukf_pose_estimation'));
addpath(fullfile(controllerRoot, 'tools'));

teacher = makeHighFidelityPwmActuator();
dataset = generatePwmIdentificationDataset(teacher, struct('sampleCount', 1200, 'randomSeed', 37));
identified = trainGrayNarxForceIdentifier(dataset, teacher);
sample = load(fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat'), 'refs');

outputDir = fullfile(tempdir, 'pwm_feedback_source_export_contract');
if exist(outputDir, 'dir')
    rmdir(outputDir, 's');
end
cleanup = onCleanup(@() cleanupOutput(outputDir));
result = exportPwmFeedbackSourceComparison(outputDir, teacher, identified, sample.refs, ...
    struct('duration', 0.4));

expectedFiles = {'summary_metrics.csv', 'per_axis_metrics.csv', ...
    'error_attribution.csv', 'acceptance.csv', 'comparison.mat', 'README.md', ...
    '01_three_line_spatial_trajectory.png', '02_pose_error_time_history.png', ...
    '03_per_axis_pose_errors.png', '04_core_metrics.png', ...
    '05_force_estimation_comparison.png', '06_force_pwm_comparison.png', ...
    '07_error_attribution.png', '08_critical_moments_zoom.png'};
for index = 1:numel(expectedFiles)
    fileName = fullfile(outputDir, expectedFiles{index});
    assert(isfile(fileName));
    fileInfo = dir(fileName);
    assert(fileInfo.bytes > 0);
end
assert(height(result.summaryMetrics) == 3);
assert(any(result.summaryMetrics.line == "identifiedTruthFeedback"));
assert(isfield(result.comparison, 'identifiedTruthFeedback'));
end

function cleanupOutput(outputDir)
if exist(outputDir, 'dir')
    rmdir(outputDir, 's');
end
end
