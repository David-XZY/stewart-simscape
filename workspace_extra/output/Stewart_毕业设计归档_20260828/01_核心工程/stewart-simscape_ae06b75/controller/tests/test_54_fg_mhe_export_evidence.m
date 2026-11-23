function test_54_fg_mhe_export_evidence
% test_54_fg_mhe_export_evidence - 验证 SC-FG-MHE 图表和表格导出
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

outputDir = fullfile(tempdir, ['fg_mhe_export_', char(java.util.UUID.randomUUID)]);
comparison = compareUkfAndFgMheEstimation(struct('smoke', true));
exported = exportFgMheEvidence(comparison, struct('outputDir', outputDir));

assert(exist(fullfile(outputDir, 'fg_mhe_results.mat'), 'file') == 2);
assert(exist(fullfile(outputDir, 'table_estimation_comparison.csv'), 'file') == 2);
assert(exist(fullfile(outputDir, 'table_ablation_factor_graph.csv'), 'file') == 2);
assert(numel(exported.figureFilesPng) >= 12);
assert(numel(exported.figureFilesFig) >= 12);
for index = 1:12
    assert(exist(exported.figureFilesPng{index}, 'file') == 2);
    assert(exist(exported.figureFilesFig{index}, 'file') == 2);
end
end
