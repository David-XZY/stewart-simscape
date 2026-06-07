function configureSimscapeGravity(modelName, gravity, args)
% configureSimscapeGravity - 在当前已加载模型中启用与优化模型一致的重力
%
% 本函数只修改内存中的模型参数。调用方关闭模型时不保存，避免把场景参数
% 固化到通用 Simscape 模型文件。

arguments
    modelName char
    gravity (3,1) double
    args.enabled (1,1) logical = true
end

block = [modelName, '/Simscape Configuration/Mechanism Configuration'];
if args.enabled
    set_param(block, ...
        'UniformGravity', 'Constant', ...
        'GravityVector', mat2str(gravity.'));
else
    set_param(block, 'UniformGravity', 'None');
end
end
