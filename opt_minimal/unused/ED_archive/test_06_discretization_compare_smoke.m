%% test_06_discretization_compare_smoke - 离散方法比较入口小规模烟雾测试
% 用途：
%   使用很小网格验证 run_03 比较链路可以完成构建、求解/失败记录、重建和汇总输出，
%   防止某一种方法失败导致整个实验脚本中断。
clear; clc;
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));

run3File = fullfile(projectRoot, 'opt_minimal', 'run_03_compare_CHSID_CHSED_DMSID_DMSED.m');
run3Source = fileread(run3File);
assert(contains(run3Source, "parser.addParameter('gridList', [20 10; 40 20; 60 30; 80 40]);"), ...
    'run3 默认网格必须保留 [20 10; 40 20; 60 30; 80 40]，禁用实验方法不能缩减网格。');

result = run_03_compare_CHSID_CHSED_DMSID_DMSED('gridList', [2 1], 'makePlots', false, 'makeAnimation', false, 'maxIter', 5);
fprintf('TEST_06 discretization compare smoke\n');
fprintf('rows=%d resultDir=%s\n', numel(result.rows), result.resultDir);
assert(numel(result.rows) == 2, '小规模烟雾测试必须记录默认启用的 CHSID/DMSID 两行结果。');
assert(exist(fullfile(result.resultDir, 'comparison_summary.csv'), 'file') == 2, '缺少 comparison_summary.csv。');
fprintf('TEST_06_OK\n');
