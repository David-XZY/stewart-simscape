%% test_15_midterm_report_figures_contract - 中期报告补图入口契约测试
% 使用既有比较结果生成报告图，不重新运行轨迹优化。

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));

comparisonMatFile = fullfile(projectRoot, 'opt_minimal', 'results', ...
    'compare_CHSID_IHSID_DMSID_20260531_125756', 'comparison_results.mat');
assert(isfile(comparisonMatFile), '测试所需比较结果 MAT 文件不存在。');
assert(exist('writeMidtermReportFigures', 'file') == 2, ...
    '必须提供独立的中期报告绘图入口 writeMidtermReportFigures。');

outputDir = tempname;
cleanup = onCleanup(@() cleanupOutput(outputDir)); %#ok<NASGU>
result = writeMidtermReportFigures(comparisonMatFile, outputDir);

expectedBases = ["fig4_method_performance_feasibility", "fig7_ihsid_engineering_constraints"];
assert(isequal(string(result.figureBaseNames(:)).', expectedBases), ...
    '必须只输出修改后保留的图4和图7。');
assert(all(isfile(result.pngFiles)) && all(isfile(result.pdfFiles)), ...
    '保留的两幅图必须同时输出 PNG 和 PDF。');
assert(numel(result.figureHandles) == 2 && all(isgraphics(result.figureHandles, 'figure')), ...
    '绘图入口必须保留可见的 figure 窗口。');
assert(all(string(get(result.figureHandles, 'Visible')) == "on"), ...
    '绘图入口生成的 figure 窗口必须可见。');
assert(result.selectedTrial.method == "IHSID" && ...
    result.selectedTrial.N1 == 60 && result.selectedTrial.N2 == 30, ...
    '最佳轨迹必须自动选择 IHSID 60+30。');
assert(result.selectedTrial.solverSuccess && result.selectedTrial.engineeringPassed, ...
    '选中的 IHSID 60+30 必须求解成功并通过工程后验。');
assert(~any(contains(string(result.figureMethods), "DMSID")), ...
    '正文图不得包含 DMSID。');
assert(result.chsedData.source == "表4.docx 表1", ...
    'CHSED 数值来源必须明确记录为文档表1。');
assert(result.chsedData.numIneq == 3847, ...
    'CHSED 20+10 网格的不等式约束数量必须按现有离散结构计算为 3847。');
assert(all(result.methodNumIneq == 3847), ...
    '图4中三种方法的不等式约束数量必须均为 3847。');

close(result.figureHandles);

disp('test_15_midterm_report_figures_contract passed');

function cleanupOutput(outputDir)
if isfolder(outputDir)
    rmdir(outputDir, 's');
end
end
