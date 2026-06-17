%% run_06_train_pwm_force_identifier - 训练并验证灰箱加残差 NARX 力估计器
clearvars -except pwmIdentificationDataFile pwmTrainingDataConfigOverrides ...
    pwmRefinementTrajectoryFile pwmRefinementOverrides;
close all; clc;

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
optRoot = fullfile(projectRoot, 'opt_minimal');
controllerRoot = fullfile(projectRoot, 'controller');
addpath(fullfile(controllerRoot, 'pwm_identification'), fullfile(controllerRoot, 'ukf_pose_estimation'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
resultDir = fullfile(projectRoot, 'results', 'controller');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
if ~exist('pwmIdentificationDataFile', 'var')
    pwmIdentificationDataFile = "";
end
if ~exist('pwmTrainingDataConfigOverrides', 'var')
    pwmTrainingDataConfigOverrides = struct();
end
if ~exist('pwmRefinementTrajectoryFile', 'var') || ...
        strlength(string(pwmRefinementTrajectoryFile)) == 0
    pwmRefinementTrajectoryFile = fullfile(optRoot, 'examples', ...
        'ihsid_40x20_limited_memory', 'simscape_references.mat');
end
if ~exist('pwmRefinementOverrides', 'var')
    pwmRefinementOverrides = makePwmClosedLoopRefinementOptions();
end

dataFile = resolveLatestFile(pwmIdentificationDataFile, resultDir, 'pwm_identification_dataset_*.mat');
if strlength(dataFile) == 0
    teacher = makeHighFidelityPwmActuator();
    dataset = generatePwmIdentificationDataset(teacher, pwmTrainingDataConfigOverrides);
else
    sample = load(dataFile, 'teacher', 'dataset');
    teacher = sample.teacher;
    dataset = sample.dataset;
end
initialIdentified = trainGrayNarxForceIdentifier(dataset, teacher);
trajectory = load(pwmRefinementTrajectoryFile, 'refs');
identified = refineGrayNarxForceIdentifierWithControlRollout( ...
    dataset, teacher, initialIdentified, trajectory.refs, pwmRefinementOverrides);
report = evaluateGrayNarxForceIdentifier(identified, dataset);

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
resultFile = fullfile(resultDir, ['pwm_force_identifier_', timestamp, '.mat']);
save(resultFile, 'teacher', 'dataset', 'initialIdentified', 'identified', 'report', ...
    'dataFile', 'pwmRefinementTrajectoryFile', 'pwmRefinementOverrides');
fprintf('\n灰箱加残差 NARX 训练结果：%s\n', resultFile);
fprintf('灰箱/NARX 测试 NRMSE：%.4f / %.4f，偏差：%.4f，通过：%d\n', ...
    report.metrics.grayTestNrmse, report.metrics.narxTestNrmse, ...
    report.metrics.testBias, report.passed);
if ~report.passed
    error('run_06_train_pwm_force_identifier:AcceptanceFailed', '辨识模型未通过验收。');
end

function fileName = resolveLatestFile(requestedFile, resultDir, pattern)
fileName = string(requestedFile);
if strlength(fileName) > 0
    if ~isfile(fileName)
        error('run_06_train_pwm_force_identifier:FileNotFound', '未找到辨识数据文件：%s', fileName);
    end
    return;
end
candidates = dir(fullfile(resultDir, pattern));
if isempty(candidates)
    fileName = "";
    return;
end
[~, index] = max([candidates.datenum]);
fileName = string(fullfile(candidates(index).folder, candidates(index).name));
end
