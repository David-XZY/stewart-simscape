%% run_08_export_pwm_feedback_source_comparison - 导出 PWM 反馈源消融组会结果
clearvars -except pwmFeedbackOutputDir pwmFeedbackOverrides;
close all; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(optRoot, 'tools'));
if ~exist('pwmFeedbackOutputDir', 'var')
    pwmFeedbackOutputDir = "";
end
if ~exist('pwmFeedbackOverrides', 'var')
    pwmFeedbackOverrides = struct();
end

result = exportPwmFeedbackSourceComparison( ...
    string(pwmFeedbackOutputDir), struct(), struct(), struct(), pwmFeedbackOverrides);
fprintf('组会结果目录：%s\n', result.outputDir);
