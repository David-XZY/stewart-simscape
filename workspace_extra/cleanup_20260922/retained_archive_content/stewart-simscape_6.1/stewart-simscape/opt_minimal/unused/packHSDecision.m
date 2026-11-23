function z = packHSDecision(Xnode, Xmid, Unode, Umid)
% packHSDecision - 打包状态/控制 Hermite-Simpson 决策变量
%
% 文件用途：
%   将节点状态、显式中点状态、节点控制和中点控制按统一顺序打包为
%   fmincon 决策变量。
%
% 输入参数：
%   Xnode [12xN]     节点状态，x=[q;qd]。
%   Xmid  [12x(N-1)] 显式中点状态。
%   Unode [6xN]      节点支链驱动力控制。
%   Umid  [6x(N-1)]  中点支链驱动力控制。
%
% 输出参数：
%   z [738x1] - 当 N=21 时的决策变量。
%
% 核心公式：
%   z=[Xnode(:);Xmid(:);Unode(:);Umid(:)]。
%
% 在优化链路中的作用：
%   主脚本、目标函数和约束函数通过本函数统一打包变量，避免维度错位。

validateattributes(Xnode, {'double'}, {'real', 'finite', '2d', 'nrows', 12}, mfilename, 'Xnode');
nodeCount = size(Xnode, 2);
validateattributes(Xmid, {'double'}, {'real', 'finite', 'size', [12, nodeCount - 1]}, mfilename, 'Xmid');
validateattributes(Unode, {'double'}, {'real', 'finite', 'size', [6, nodeCount]}, mfilename, 'Unode');
validateattributes(Umid, {'double'}, {'real', 'finite', 'size', [6, nodeCount - 1]}, mfilename, 'Umid');

z = [Xnode(:); Xmid(:); Unode(:); Umid(:)];
end
