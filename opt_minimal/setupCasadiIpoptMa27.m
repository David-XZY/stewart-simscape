function info = setupCasadiIpoptMa27(projectRoot)
% setupCasadiIpoptMa27 - 配置 CasADi/IPOPT/MA27 并执行强制预检
%
% 用途：
%   将工程自带 CasADi 3.5.5 MATLAB 包和 CoinHSL DLL 目录加入当前 MATLAB 进程，
%   使用 opts.ipopt.linear_solver='ma27' 求解一维测试 NLP。预检失败即抛出真实异常，
%   禁止静默回退到 mumps、sqp 或其他线性求解器。
%
% 输入：
%   projectRoot - stewart-simscape 工程根目录；省略时由本文件位置反推
%
% 输出：
%   info struct - CasADi 根目录、HSL bin 目录、版本、测试解、IPOPT 状态和耗时
%
% 核心公式：
%   min (x-1)^2, x in R，solver=nlpsol(...,'ipopt',opts)，opts.ipopt.linear_solver='ma27'。
%
% 优化链路位置：
%   默认入口 run_01_hs_dynamic_opt 在构建 Stewart 大 NLP 前必须先调用本函数。

if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
end

casadiRoot = fullfile(projectRoot, 'lib', 'casadi-windows-matlabR2016a-v3.5.5');
hslBin = fullfile(projectRoot, 'lib', ...
    'CoinHSL-archive.v2023.11.17.x86_64-w64-mingw32-libgfortran5', 'bin');

if exist(casadiRoot, 'dir') ~= 7
    error('setupCasadiIpoptMa27:MissingCasadi', '未找到 CasADi 目录：%s', casadiRoot);
end
if exist(hslBin, 'dir') ~= 7
    error('setupCasadiIpoptMa27:MissingHSL', '未找到 CoinHSL bin 目录：%s', hslBin);
end

addpath(casadiRoot);
setenv('PATH', [casadiRoot, pathsep, hslBin, pathsep, getenv('PATH')]);
import casadi.*

versionText = casadi.CasadiMeta.version();
fprintf('CasADi root: %s\n', casadiRoot);
fprintf('CoinHSL bin: %s\n', hslBin);
fprintf('CasADi version: %s\n', versionText);
fprintf('MA27 precheck: opts.ipopt.linear_solver = ma27\n');

x = MX.sym('x', 1, 1);
nlp = struct('x', x, 'f', (x - 1)^2, 'g', x);
opts = struct();
opts.print_time = false;
opts.ipopt.print_level = 0;
opts.ipopt.linear_solver = 'ma27';
opts.ipopt.max_iter = 50;
opts.ipopt.tol = 1e-10;
opts.ipopt.hessian_approximation = 'exact';

timerValue = tic;
try
    solver = nlpsol('ma27_precheck_solver', 'ipopt', nlp, opts);
    sol = solver('x0', 0, 'lbg', -inf, 'ubg', inf);
    stats = solver.stats();
catch ME
    error('setupCasadiIpoptMa27:PrecheckFailed', ...
        'CasADi/IPOPT/MA27 一维预检失败，未进入 Stewart NLP。原始错误：%s', ME.message);
end
elapsed = toc(timerValue);

status = char(stats.return_status);
xOpt = full(sol.x);
if abs(xOpt - 1) > 1e-7
    error('setupCasadiIpoptMa27:BadPrecheckSolution', ...
        'MA27 预检求解值异常：x=%.16g，期望 1。', xOpt);
end

fprintf('MA27 precheck success: x=%.12g, return_status=%s, time=%.3f s\n', xOpt, status, elapsed);

info = struct();
info.projectRoot = projectRoot;
info.casadiRoot = casadiRoot;
info.hslBin = hslBin;
info.casadiVersion = versionText;
info.linearSolver = 'ma27';
info.precheckX = xOpt;
info.returnStatus = status;
info.elapsed = elapsed;
info.stats = stats;
end
