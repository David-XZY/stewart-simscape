function test_03_directory_contract
% test_03_directory_contract - 验证根目录、活动依赖与过时方法残留
optRoot = fileparts(fileparts(mfilename('fullpath')));
rootMFiles = dir(fullfile(optRoot, '*.m'));
expectedEntries = {'run_01_ihsid_trajectory.m', 'run_02_simscape_length_control.m', ...
    'run_03_simscape_length_cascade_control.m', 'run_04_simscape_pose_length_control.m', ...
    'run_05_generate_pwm_identification_data.m', 'run_06_train_pwm_force_identifier.m', ...
    'run_07_compare_pwm_pose_force_control.m', ...
    'run_08_export_pwm_feedback_source_comparison.m', ...
    'run_09_export_four_control_group_meeting_comparison.m'};
assert(isequal({rootMFiles.name}, expectedEntries), '根目录只能保留轨迹与三种控制主入口。');

activeDirs = {'core', 'ihsid', 'validation', 'integration', 'actuator_identification', 'tools'};
for index = 1:numel(activeDirs)
    files = dir(fullfile(optRoot, activeDirs{index}, '**', '*.m'));
    for fileIndex = 1:numel(files)
        text = fileread(fullfile(files(fileIndex).folder, files(fileIndex).name));
        assert(isempty(regexp(text, 'unused[\\/]', 'once')), '活动源码不得调用 unused/。');
        assert(isempty(regexp(text, 'CHSID|DMSID|CHSED|DMSED|FATROP|run_01_hs_dynamic_opt', 'once')), ...
            '活动源码存在非主线方法或旧入口残留。');
    end
end

for entryIndex = 1:numel(expectedEntries)
    entryText = fileread(fullfile(optRoot, expectedEntries{entryIndex}));
    assert(isempty(regexp(entryText, 'unused[\\/]|CHSID|DMSID|CHSED|DMSED|FATROP|run_01_hs_dynamic_opt', 'once')), ...
        '主入口存在非主线方法、unused 调用或旧入口残留。');
end
end
