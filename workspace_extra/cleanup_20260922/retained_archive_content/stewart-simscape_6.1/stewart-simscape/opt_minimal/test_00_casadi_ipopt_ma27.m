%% test_00_casadi_ipopt_ma27 - CasADi/IPOPT/MA27 环境预检
% 用途：独立验证工程自带 CasADi、IPOPT 和 MA27 是否能完成一维 NLP。
% 输入：无。输出：控制台打印预检状态，失败时抛出真实错误。
clear; clc;
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
info = setupCasadiIpoptMa27(projectRoot);
assert(strcmp(info.casadiVersion, '3.7.2'), '工程必须使用 CasADi 3.7.2。');
assert(contains(info.casadiRoot, 'casadi-windows-matlabR2018b-v3.7.2'), ...
    '工程必须使用 lib 中的 CasADi 3.7.2 MATLAB 包。');
assert(info.hasFatrop, 'CasADi 3.7.2 必须能够加载 fatrop NLP 求解器插件。');
fprintf('TEST_00_OK linear_solver=%s return_status=%s x=%.12g\n', ...
    info.linearSolver, info.returnStatus, info.precheckX);
