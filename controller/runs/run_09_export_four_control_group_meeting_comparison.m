%% run_09_export_four_control_group_meeting_comparison - 导出四种控制方案中文组会对比图
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
optRoot = fullfile(projectRoot, 'opt_minimal');
controllerRoot = fullfile(projectRoot, 'controller');
addpath(fullfile(controllerRoot, 'tools'));

result = exportFourControlGroupMeetingComparison();
fprintf('组会对比图目录：%s\n', result.outputDir);
