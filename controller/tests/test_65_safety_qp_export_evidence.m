function test_65_safety_qp_export_evidence
% test_65_safety_qp_export_evidence - 验证 SC-QP 图表与论文表格导出
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'safety_qp_control'));

model = buildOptModelCustom();
config = makeSafetyQpControlConfig(model, struct('dt', 0.02));
refs = makeShortReference(model, config);
comparison = compareSafetyQpWithBaselineControllers(refs, model, config, struct('fastMode', true));

outputDir = fullfile(tempdir, 'safety_qp_export_contract');
if isfolder(outputDir)
    rmdir(outputDir, 's');
end
cleanup = onCleanup(@() removeDirectory(outputDir));

exported = exportSafetyQpEvidence(comparison, outputDir, config);

assert(isfile(fullfile(outputDir, 'table_control_comparison.csv')));
assert(isfile(fullfile(outputDir, 'table_clf_cbf_ablation.csv')));
assert(isfile(fullfile(outputDir, 'table_robustness_safety_qp.csv')));
assert(isfile(fullfile(outputDir, 'safety_qp_comparison_data.mat')));
pngFiles = dir(fullfile(outputDir, '*.png'));
figFiles = dir(fullfile(outputDir, '*.fig'));
assert(numel(pngFiles) >= 15);
assert(numel(figFiles) >= 15);
assert(numel(exported.figureFiles) >= 15);

controlTable = readtable(fullfile(outputDir, 'table_control_comparison.csv'), ...
    'TextType', 'string');
requiredColumns = [
    "method"
    "position_rmse"
    "attitude_rmse"
    "max_position_error"
    "max_attitude_error"
    "max_force"
    "max_force_rate"
    "control_energy"
    "min_sigma"
    "max_condition_number"
    "min_collision_distance"
    "min_cbf_margin"
    "mean_qp_time"
    "qp_infeasible_count"
    "safety_brake_count"
    "improvement_over_lqi_percent"
    ];
assert(all(ismember(requiredColumns, string(controlTable.Properties.VariableNames).')));
assert(height(controlTable) >= 9);
end

function refs = makeShortReference(model, config)
time = 0:config.dt:0.12;
sampleCount = numel(time);
shape = sin(pi * time / time(end));
refs = struct();
refs.t = time;
refs.q = model.qHome + [0.002; -0.001; 0.001; 0.001; -0.0005; 0.0008] .* shape;
refs.qd = differentiateRows(refs.q, time);
refs.qdd = differentiateRows(refs.qd, time);
refs.Fleg = repmat(0.5 * ones(6, 1), 1, sampleCount);
refs.Fcomputed = repmat(0.6 * ones(6, 1), 1, sampleCount);
refs.L = zeros(6, sampleCount);
refs.Ld = zeros(6, sampleCount);
for index = 1:sampleCount
    refs.L(:, index) = sgpIK(refs.q(:, index), model).L;
    refs.Ld(:, index) = sgpJacobian(refs.q(:, index), model).Jq * refs.qd(:, index);
end
end

function derivative = differentiateRows(value, time)
derivative = zeros(size(value));
for row = 1:size(value, 1)
    derivative(row, :) = gradient(value(row, :), time);
end
end

function removeDirectory(outputDir)
if isfolder(outputDir)
    rmdir(outputDir, 's');
end
end
