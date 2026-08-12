%% run_10_factor_graph_parameter_estimation - 运行 SC-FG-MHE 参数估计主线
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
controllerRoot = fullfile(projectRoot, 'controller');
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'pwm_identification'));
addpath(fullfile(controllerRoot, 'ukf_pose_estimation'));
addpath(fullfile(controllerRoot, 'factor_graph_estimation'));

comparison = compareUkfAndFgMheEstimation(struct('smoke', false));
exported = exportFgMheEvidence(comparison);

fprintf('SC-FG-MHE 结果目录：%s\n', exported.outputDir);
fprintf('对比表：%s\n', exported.metricsCsv);
fprintf('消融表：%s\n', exported.ablationCsv);
