function plotFiles = writeDiscretizationComparisonFigures(resultMatFile)
% writeDiscretizationComparisonFigures - 为离散比较结果生成 PNG 图
%
% 输入：
%   resultMatFile - run_03 输出的 comparison_results.mat。
%
% 输出：
%   plotFiles - 生成的 PNG 文件路径。
loaded = load(resultMatFile, 'summaryTable', 'trials');
summaryTable = loaded.summaryTable;
trials = loaded.trials;
resultDir = fileparts(resultMatFile);
plotFiles = {};
if isempty(summaryTable)
    return;
end
gridKey = chooseGrid(summaryTable);
rows = summaryTable(summaryTable.N1 == gridKey(1) & summaryTable.N2 == gridKey(2), :);
gridTrials = trials(arrayfun(@(x) x.disc.numIntervalsApproach == gridKey(1) && ...
    x.disc.numIntervalsInsertion == gridKey(2) && ~isempty(x.traj), trials));

fig = figure('Name', 'comparison_table', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; bar(categorical(rows.method), rows.solveTime_s); ylabel('求解时间 [s]'); grid on;
nexttile; bar(categorical(rows.method), rows.objectiveTotal); ylabel('目标函数'); grid on;
nexttile; bar(categorical(rows.method), rows.minStage1Clearance_m); ylabel('第一阶段最小间隙 [m]'); grid on;
nexttile; bar(categorical(rows.method), rows.maxDynResidual); ylabel('动力学残差'); grid on;
plotFiles{end+1} = saveFig(fig, resultDir, sprintf('comparison_table_N%d_%d.png', gridKey)); %#ok<AGROW>

if ~isempty(gridTrials)
    fig = figure('Name', 'trajectory_overlay', 'Color', 'w'); hold on; grid on; axis equal;
    for i = 1:numel(gridTrials)
        traj = gridTrials(i).traj;
        plot3(traj.Q(1,:), traj.Q(2,:), traj.Q(3,:), 'LineWidth', 1.2, 'DisplayName', gridTrials(i).method);
    end
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]'); legend('Location', 'best');
    plotFiles{end+1} = saveFig(fig, resultDir, sprintf('trajectory_overlay_N%d_%d.png', gridKey)); %#ok<AGROW>

    fig = figure('Name', 'clearance_compare', 'Color', 'w'); hold on; grid on;
    for i = 1:numel(gridTrials)
        traj = gridTrials(i).traj;
        plot(traj.t, traj.minClearance, 'LineWidth', 1.2, 'DisplayName', gridTrials(i).method);
    end
    xlabel('t [s]'); ylabel('节点最小间隙 [m]'); legend('Location', 'best');
    plotFiles{end+1} = saveFig(fig, resultDir, sprintf('clearance_compare_N%d_%d.png', gridKey)); %#ok<AGROW>

    fig = figure('Name', 'constraint_compare', 'Color', 'w'); hold on; grid on;
    for i = 1:numel(gridTrials)
        traj = gridTrials(i).traj;
        plot(traj.t, max(abs(traj.Ld), [], 1), 'LineWidth', 1.2, 'DisplayName', [gridTrials(i).method ' |Ld|']);
    end
    xlabel('t [s]'); ylabel('最大绝对腿速 [m/s]'); legend('Location', 'best');
    plotFiles{end+1} = saveFig(fig, resultDir, sprintf('constraint_compare_N%d_%d.png', gridKey)); %#ok<AGROW>
end
end

function gridKey = chooseGrid(summaryTable)
if any(summaryTable.N1 == 40 & summaryTable.N2 == 20)
    gridKey = [40 20];
else
    gridKey = [summaryTable.N1(1), summaryTable.N2(1)];
end
end

function fileName = saveFig(fig, resultDir, name)
fileName = fullfile(resultDir, name);
exportgraphics(fig, fileName, 'Resolution', 160);
close(fig);
end
