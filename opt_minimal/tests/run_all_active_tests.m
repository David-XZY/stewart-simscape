%% run_all_active_tests - 执行 opt_minimal IHSID 活动主线契约测试
testFiles = {
    'test_01_ihsid_contract'
    'test_02_simscape_export_contract'
    'test_03_directory_contract'
    'test_04_simscape_model_contract'
    'test_05_simscape_parameter_mapping'
    'test_06_simscape_length_control_smoke'
    'test_07_simscape_preparation_contract'
    'test_08_simscape_full_tracking'
    'test_09_length_cascade_reference_contract'
    'test_10_length_cascade_design_contract'
    'test_11_length_cascade_preparation_contract'
    'test_12_length_cascade_model_contract'
    'test_13_length_cascade_evaluation_contract'
    'test_14_length_cascade_smoke'
    'test_15_length_cascade_full_tracking'
    'test_16_meeting_figure_export_contract'
    'test_17_pose_force_control_contract'
    'test_18_pose_force_preparation_contract'
    'test_19_pose_force_full_tracking'
    'test_20_hermite_reference_contract'
};

for index = 1:numel(testFiles)
    fprintf('\n===== %s =====\n', testFiles{index});
    feval(testFiles{index});
end
fprintf('\n全部 IHSID 活动主线契约测试通过。\n');
