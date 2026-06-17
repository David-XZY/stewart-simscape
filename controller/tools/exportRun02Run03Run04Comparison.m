function result = exportRun02Run03Run04Comparison(run02File, run03File, run04File, outputDir)
% exportRun02Run03Run04Comparison - 导出三种控制结构的共同误差对比图
arguments
    run02File {mustBeTextScalar} = ""
    run03File {mustBeTextScalar} = ""
    run04File {mustBeTextScalar} = ""
    outputDir {mustBeTextScalar} = ""
end

controllerRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
resultRoot = fullfile(projectRoot, 'results', 'controller');
run02File = chooseFile(run02File, resultRoot, 'result_simscape_pose_force_control_*.mat');
run03File = chooseFile(run03File, resultRoot, 'result_simscape_length_cascade_*.mat');
run04File = chooseFile(run04File, resultRoot, 'result_simscape_pose_length_control_*.mat');
if strlength(string(outputDir)) == 0
    outputDir = fullfile(resultRoot, 'run02_run03_run04_comparison');
end
outputDir = char(outputDir);
if ~isfolder(outputDir)
    mkdir(outputDir);
end

runs = {load(run02File), load(run03File), load(run04File)};
labels = {'位姿反馈力输入', '纯腿长串级控制', '位姿反馈腿长输入'};
colors = [0.00 0.35 0.70; 0.90 0.40 0.05; 0.00 0.55 0.35];
for index = 1:3
    assert(isfield(runs{index}, 'report') && runs{index}.report.passed, ...
        '%s 结果未通过硬验收。', labels{index});
end

fig = figure('Color', 'w', 'Visible', 'off', 'Units', 'pixels', ...
    'Position', [80, 80, 1600, 900]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
sgtitle(layout, '三种控制结构跟踪性能对比', 'FontSize', 18, 'FontWeight', 'bold');
plotPeakHistory(nexttile(layout), runs, labels, colors, 'length', '逐时刻最大腿长误差', '误差 (mm)');
plotPeakHistory(nexttile(layout), runs, labels, colors, 'translation', '逐时刻最大平移误差', '误差 (mm)');
plotPeakHistory(nexttile(layout), runs, labels, colors, 'rotation', '逐时刻最大转角误差', '误差 (deg)');

metrics = zeros(3, 4);
for index = 1:3
    report = runs{index}.report;
    equivalent = [report.poseError(:, 1:3), 0.5 * report.poseError(:, 4:6)];
    metrics(index, :) = [sqrt(mean(equivalent.^2, 'all')) * 1e3, ...
        max(abs(equivalent), [], 'all') * 1e3, ...
        report.metrics.maxTranslationPeak * 1e3, ...
        rad2deg(report.metrics.maxRotationPeak)];
end
ax = nexttile(layout);
bars = bar(ax, metrics.', 'grouped');
for index = 1:3
    bars(index).FaceColor = colors(index, :);
end
set(ax, 'XTickLabel', {'等效位姿 RMS (mm)', '等效位姿峰值 (mm)', ...
    '平移峰值 (mm)', '转角峰值 (deg)'});
grid(ax, 'on'); title(ax, '共同位姿指标'); legend(ax, labels, 'Location', 'best');

figFile = fullfile(outputDir, 'run02_run03_run04_tracking_comparison.fig');
pngFile = fullfile(outputDir, 'run02_run03_run04_tracking_comparison.png');
savefig(fig, figFile);
exportgraphics(fig, pngFile, 'Resolution', 300);
close(fig);

result = struct('run02File', char(run02File), 'run03File', char(run03File), ...
    'run04File', char(run04File), 'figFile', figFile, 'pngFile', pngFile, ...
    'labels', {labels}, 'metrics', metrics);
end

function plotPeakHistory(ax, runs, labels, colors, quantity, titleText, yLabel)
for index = 1:3
    report = runs{index}.report;
    switch quantity
        case 'length'
            time = report.time;
            value = max(abs(report.lengthError), [], 2) * 1e3;
        case 'translation'
            time = report.poseTime;
            value = max(abs(report.poseError(:, 1:3)), [], 2) * 1e3;
        case 'rotation'
            time = report.poseTime;
            value = rad2deg(max(abs(report.poseError(:, 4:6)), [], 2));
    end
    plot(ax, time, value, 'Color', colors(index, :), 'LineWidth', 1.4);
    hold(ax, 'on');
end
grid(ax, 'on'); title(ax, titleText); xlabel(ax, '时间 (s)'); ylabel(ax, yLabel);
legend(ax, labels, 'Location', 'best');
end

function fileName = chooseFile(requestedFile, resultRoot, pattern)
if strlength(string(requestedFile)) > 0
    fileName = char(requestedFile);
else
    files = dir(fullfile(resultRoot, pattern));
    assert(~isempty(files), '未找到结果文件：%s', pattern);
    [~, index] = max([files.datenum]);
    fileName = fullfile(files(index).folder, files(index).name);
end
assert(isfile(fileName), '结果文件不存在：%s', fileName);
end
