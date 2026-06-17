function test_39_four_control_group_meeting_export_contract
% test_39_four_control_group_meeting_export_contract - 验证四方案中文组会图导出契约
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(controllerRoot, 'tools'));

outputDir = fullfile(tempdir, 'four_control_group_meeting_export_contract');
if isfolder(outputDir)
    rmdir(outputDir, 's');
end
cleanup = onCleanup(@() removeDirectory(outputDir));

result = exportFourControlGroupMeetingComparison("", "", outputDir);

expectedFiles = {
    '四方案总体指标.csv'
    '四方案分轴指标.csv'
    '四方案对比结果.mat'
    'README.md'
    '01_四方案核心性能总览.png'
    '02_六自由度分轴误差对比.png'
    '03_轨迹与关键时刻局部放大.png'
    '04_控制负担对比.png'
    '05_逐级性能差距图.png'
    };
for index = 1:numel(expectedFiles)
    fileName = fullfile(outputDir, expectedFiles{index});
    assert(isfile(fileName));
    assert(dir(fileName).bytes > 0);
end

assert(height(result.summaryMetrics) == 4);
assert(isequal(result.summaryMetrics.label, ...
    ["纯力控制"; "高保真控制"; "真实位姿反馈辨识控制"; "UKF反馈辨识控制"]));
assert(all(isfinite(result.summaryMetrics.translationRmsMm)));
assert(isnan(result.summaryMetrics.maxAbsPwm(1)));
assert(all(result.summaryMetrics.pwmApplicable == [false; true; true; true]));
assert(numel(result.runs) == 4);
assert(all(arrayfun(@(run) isequal(run.time, result.runs(1).time), result.runs)));
assert(norm(result.runs(1).referencePose(:, 1) - result.runs(2).referencePose(:, 1)) < 1e-12);

pngFiles = dir(fullfile(outputDir, '*.png'));
pdfFiles = dir(fullfile(outputDir, '*.pdf'));
figFiles = dir(fullfile(outputDir, '*.fig'));
assert(numel(pngFiles) == 5);
assert(numel(pdfFiles) == 5);
assert(numel(figFiles) == 5);
for index = 1:numel(pngFiles)
    imageInfo = imfinfo(fullfile(pngFiles(index).folder, pngFiles(index).name));
    assert(abs(imageInfo.Width / imageInfo.Height - 16 / 9) < 0.03);
    assert(imageInfo.Width >= 4000);
end

readmeText = fileread(fullfile(outputDir, 'README.md'));
assert(contains(readmeText, '纯力控制'));
assert(contains(readmeText, '理想力源系统基线'));
end

function removeDirectory(outputDir)
if isfolder(outputDir)
    rmdir(outputDir, 's');
end
end
