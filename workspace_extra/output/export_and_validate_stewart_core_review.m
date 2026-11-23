packageRoot = string(getenv('STEWART_CORE_PACKAGE_ROOT'));
if strlength(packageRoot) == 0 || ~isfolder(packageRoot)
    error('StewartCoreReview:MissingPackageRoot', ...
        'STEWART_CORE_PACKAGE_ROOT is missing or invalid.');
end

codeRoot = fullfile(packageRoot, '01_核心代码', 'stewart-simscape');
resultRoot = fullfile(packageRoot, '02_精选结果');
traceRoot = fullfile(packageRoot, '00_追溯与清单');
controlRoot = fullfile(resultRoot, '02_运动控制');
controlMat = fullfile(controlRoot, ...
    'result_simscape_pose_force_control_20260614_174539.mat');

loaded = load(controlMat, 'report');
assert(isfield(loaded, 'report') && isstruct(loaded.report));
report = loaded.report;
requiredReportFields = {'time', 'poseTime', 'forceTime', ...
    'poseError', 'lengthError', 'controlForce', 'metrics', 'passed'};
for index = 1:numel(requiredReportFields)
    assert(isfield(report, requiredReportFields{index}), ...
        'Missing report field: %s', requiredReportFields{index});
end
assert(report.passed, 'The selected representative control result did not pass.');

n = size(report.poseError, 1);
assert(n == size(report.lengthError, 1));
assert(n == size(report.controlForce, 1));
assert(n == numel(report.time));
keepCount = min(n, 1000);
sampleIndex = unique(round(linspace(1, n, keepCount)));
sampleIndex = sampleIndex(:);
data = [report.time(sampleIndex), ...
    report.poseError(sampleIndex, :), ...
    report.lengthError(sampleIndex, :), ...
    report.controlForce(sampleIndex, :)];
variableNames = {'time_s', ...
    'ex_m', 'ey_m', 'ez_m', 'eroll_rad', 'epitch_rad', 'eyaw_rad', ...
    'leg1_error_m', 'leg2_error_m', 'leg3_error_m', ...
    'leg4_error_m', 'leg5_error_m', 'leg6_error_m', ...
    'leg1_force_N', 'leg2_force_N', 'leg3_force_N', ...
    'leg4_force_N', 'leg5_force_N', 'leg6_force_N'};
compactTable = array2table(data, 'VariableNames', variableNames);
compactCsv = fullfile(controlRoot, ...
    'representative_pose_force_control_compact_1000.csv');
writetable(compactTable, compactCsv);
assert(height(compactTable) <= 1000);
assert(compactTable.time_s(1) == report.time(1));
assert(compactTable.time_s(end) == report.time(end));

roundTripTable = readtable(compactCsv, 'VariableNamingRule', 'preserve');
assert(height(roundTripTable) == height(compactTable));
assert(width(roundTripTable) == width(compactTable));

trajectoryMat = fullfile(resultRoot, '01_轨迹规划', 'simscape_references.mat');
trajectoryInfo = whos('-file', trajectoryMat);
trajectoryVariables = string({trajectoryInfo.name});
assert(any(trajectoryVariables == "refs"));

experimentRoot = fullfile(resultRoot, '03_当前观测器实验');
ablation = readtable(fullfile(experimentRoot, ...
    'observed_state_ablation_summary.csv'), ...
    'VariableNamingRule', 'preserve');
assert(height(ablation) == 2);
assert(all(ablation.observedPassed == 0));

pngFiles = dir(fullfile(resultRoot, '**', '*.png'));
for index = 1:numel(pngFiles)
    imageInfo = imfinfo(fullfile(pngFiles(index).folder, pngFiles(index).name));
    assert(imageInfo.Width > 0 && imageInfo.Height > 0);
end

mFiles = dir(fullfile(codeRoot, '**', '*.m'));
staticMessageCount = 0;
for index = 1:numel(mFiles)
    messages = checkcode(fullfile(mFiles(index).folder, mFiles(index).name), '-struct');
    staticMessageCount = staticMessageCount + numel(messages);
end

modelDir = fullfile(codeRoot, 'matlab');
subsystemDir = fullfile(modelDir, 'simscape_subsystems');
addpath(modelDir, subsystemDir, fullfile(codeRoot, 'src'));
modelFile = fullfile(modelDir, 'stewart_platform_model.slx');
load_system(modelFile);
assert(bdIsLoaded('stewart_platform_model'));
close_system('stewart_platform_model', 0);

validationFile = fullfile(traceRoot, 'MATLAB_VALIDATION.txt');
fid = fopen(validationFile, 'w');
assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'MATLAB package validation\n');
fprintf(fid, 'matlabVersion: %s\n', version);
fprintf(fid, 'representativeMatReadable: 1\n');
fprintf(fid, 'representativeResultPassed: %d\n', report.passed);
fprintf(fid, 'originalRows: %d\n', n);
fprintf(fid, 'compactCsvRows: %d\n', height(compactTable));
fprintf(fid, 'compactCsvColumns: %d\n', width(compactTable));
fprintf(fid, 'trajectoryMatReadable: 1\n');
fprintf(fid, 'trajectoryRefsPresent: 1\n');
fprintf(fid, 'experimentSummaryRows: %d\n', height(ablation));
fprintf(fid, 'experimentObservedPassedAllZero: 1\n');
fprintf(fid, 'pngFilesReadable: %d\n', numel(pngFiles));
fprintf(fid, 'mFilesChecked: %d\n', numel(mFiles));
fprintf(fid, 'checkcodeMessageCount: %d\n', staticMessageCount);
fprintf(fid, 'mainModelLoadPassed: 1\n');
fprintf(fid, 'modelClosedWithoutSaving: 1\n');

disp('STEWART_CORE_REVIEW_MATLAB_VALIDATION_OK');
