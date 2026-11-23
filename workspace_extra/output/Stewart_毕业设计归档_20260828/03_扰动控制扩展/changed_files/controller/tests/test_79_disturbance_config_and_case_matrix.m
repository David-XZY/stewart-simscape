function test_79_disturbance_config_and_case_matrix
% Canonical matrix and train/holdout split.
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot, 'controller', 'disturbance_control_comparison'));
config = makeDisturbanceComparisonConfig();
allCases = buildDisturbanceCaseMatrix(config, 'all');
training = buildDisturbanceCaseMatrix(config, 'training');
holdout = buildDisturbanceCaseMatrix(config, 'holdout');
assert(numel(allCases) == 40);
assert(numel(training) == 10);
assert(numel(holdout) == 30);
assert(numel(unique([allCases.id])) == 40);
assert(isequal(config.disturbanceScales, [0.5, 1.0, 1.5]));
assert(isequal(config.noiseSeeds, 101:105));
assert(sum([allCases.hasTargetNoise]) == 30);
fprintf('test_79_disturbance_config_and_case_matrix passed.\n');
end
