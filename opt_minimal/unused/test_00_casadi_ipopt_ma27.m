%% test_00_casadi_ipopt_ma27 - CasADi/IPOPT/MA27 环境预检
% 用途：独立验证工程自带 CasADi、IPOPT 和 MA27 是否能完成一维 NLP。
% 输入：无。输出：控制台打印预检状态，失败时抛出真实错误。
clear; clc;
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
info = setupCasadiIpoptMa27(projectRoot);
fprintf('TEST_00_OK linear_solver=%s return_status=%s x=%.12g\n', ...
    info.linearSolver, info.returnStatus, info.precheckX);
