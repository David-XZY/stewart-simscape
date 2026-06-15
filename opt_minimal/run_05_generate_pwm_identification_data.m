%% run_05_generate_pwm_identification_data - 生成 PWM 执行器辨识数据
clearvars -except pwmDataConfigOverrides;
close all; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(optRoot, 'actuator_identification'));
addpath(fullfile(optRoot, 'core'));
if ~exist('pwmDataConfigOverrides', 'var')
    pwmDataConfigOverrides = struct();
end

teacher = makeHighFidelityPwmActuator();
dataset = generatePwmIdentificationDataset(teacher, pwmDataConfigOverrides);
resultDir = fullfile(optRoot, 'results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
resultFile = fullfile(resultDir, ['pwm_identification_dataset_', timestamp, '.mat']);
save(resultFile, 'teacher', 'dataset', 'pwmDataConfigOverrides');

fprintf('\nPWM 执行器辨识数据已生成：%s\n', resultFile);
fprintf('样本数：%d，采样周期：%.4f s，PWM 峰值：%.1f\n', ...
    numel(dataset.time), dataset.sampleTime, max(abs(dataset.pwm), [], 'all'));
