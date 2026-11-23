function test_53_fg_mhe_compare_with_ukf_smoke
% test_53_fg_mhe_compare_with_ukf_smoke - 验证 UKF 与 SC-FG-MHE 对比 smoke 路径
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'pwm_identification'));
addpath(fullfile(controllerRoot, 'ukf_pose_estimation'));
addpath(fullfile(controllerRoot, 'factor_graph_estimation'));

comparison = compareUkfAndFgMheEstimation(struct('smoke', true));
methodNames = string(comparison.metrics.method);
expected = ["kinematic", "ukf_stable", "ukf_leg_bias", ...
    "fg_mhe_no_dynamics", "fg_mhe_with_dynamics", ...
    "fg_mhe_robust", "fg_mhe_actuator"];
for index = 1:numel(expected)
    assert(any(methodNames == expected(index)));
end

requiredColumns = ["method", "position_rmse", "attitude_rmse", ...
    "leg_bias_rmse", "payload_mass_error", "payload_com_error", ...
    "actuator_gain_error", "mean_solve_time", "failure_count", ...
    "improvement_over_ukf_percent"];
for index = 1:numel(requiredColumns)
    assert(any(string(comparison.metrics.Properties.VariableNames) == requiredColumns(index)));
end
end
