%% run_all_active_tests - 执行 opt_minimal IHSID 活动主线契约测试
testFiles = {
    'test_01_ihsid_contract'
    'test_02_simscape_export_contract'
    'test_03_directory_contract'
    'test_04_simscape_model_contract'
};

for index = 1:numel(testFiles)
    fprintf('\n===== %s =====\n', testFiles{index});
    feval(testFiles{index});
end
fprintf('\n全部 IHSID 活动主线契约测试通过。\n');
