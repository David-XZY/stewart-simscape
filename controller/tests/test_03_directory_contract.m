function test_03_directory_contract
% test_03_directory_contract - 验证轨迹规划与控制代码目录契约
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');

rootMFiles = dir(fullfile(optRoot, '*.m'));
expectedOptEntries = {'run_01_ihsid_trajectory.m'};
assert(isequal({rootMFiles.name}, expectedOptEntries), ...
    'opt_minimal 根目录只保留轨迹规划主入口。');

controllerEntries = {'run_02_simscape_length_control.m', ...
    'run_03_simscape_length_cascade_control.m', 'run_04_simscape_pose_length_control.m', ...
    'run_05_generate_pwm_identification_data.m', 'run_06_train_pwm_force_identifier.m', ...
    'run_07_compare_pwm_pose_force_control.m', ...
    'run_08_export_pwm_feedback_source_comparison.m', ...
    'run_09_export_four_control_group_meeting_comparison.m'};
controllerRunFiles = dir(fullfile(controllerRoot, 'runs', '*.m'));
assert(isequal({controllerRunFiles.name}, controllerEntries), ...
    'controller/runs 只保留控制与汇报导出入口。');

activeDirs = {
    fullfile(optRoot, 'core')
    fullfile(optRoot, 'ihsid')
    fullfile(optRoot, 'validation')
    fullfile(controllerRoot, 'simscape_tracking')
    fullfile(controllerRoot, 'pwm_identification')
    fullfile(controllerRoot, 'ukf_pose_estimation')
    fullfile(controllerRoot, 'tools')
};
for index = 1:numel(activeDirs)
    files = dir(fullfile(activeDirs{index}, '**', '*.m'));
    for fileIndex = 1:numel(files)
        text = fileread(fullfile(files(fileIndex).folder, files(fileIndex).name));
        assert(isempty(regexp(text, 'unused[\\/]', 'once')), ...
            '活动源码不得调用 unused/。');
        assert(isempty(regexp(text, 'CHSID|DMSID|CHSED|DMSED|FATROP|run_01_hs_dynamic_opt', 'once')), ...
            '活动源码不得残留非主线方法或旧入口。');
    end
end

for entryIndex = 1:numel(expectedOptEntries)
    entryText = fileread(fullfile(optRoot, expectedOptEntries{entryIndex}));
    assert(isempty(regexp(entryText, 'unused[\\/]|CHSID|DMSID|CHSED|DMSED|FATROP|run_01_hs_dynamic_opt', 'once')), ...
        '轨迹规划主入口不得调用 unused 或旧方法。');
end

for entryIndex = 1:numel(controllerEntries)
    entryText = fileread(fullfile(controllerRoot, 'runs', controllerEntries{entryIndex}));
    assert(isempty(regexp(entryText, 'unused[\\/]|CHSID|DMSID|CHSED|DMSED|FATROP|run_01_hs_dynamic_opt', 'once')), ...
        '控制主入口不得调用 unused 或旧方法。');
end
end
