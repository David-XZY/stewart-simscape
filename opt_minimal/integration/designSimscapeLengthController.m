function design = designSimscapeLengthController(modelName, bandwidthHz)
% designSimscapeLengthController - 线性化力到腿长对象并整定对角 PIDF
%
% 控制器仅使用腿长误差。对角控制保留现有 Reference-Tracking-L 结构，
% 不引入优化力前馈或位姿反馈。

arguments
    modelName char = 'stewart_platform_model'
    bandwidthHz (1,1) double {mustBePositive} = 0.5
end

options = linearizeOptions;
options.SampleTime = 0;
io(1) = linio([modelName, '/Controller'], 1, 'openinput');
io(2) = linio([modelName, '/Stewart Platform'], 1, 'openoutput', [], 'dLm');
plant = linearize(modelName, io, options);

if ~isequal(size(plant), [6, 6])
    error('designSimscapeLengthController:InvalidPlantSize', ...
        '力到腿长线性对象必须为 6x6，当前为 %dx%d。', size(plant, 1), size(plant, 2));
end

legControllers = cell(1, 6);
for legIndex = 1:6
    legControllers{legIndex} = pidtune( ...
        plant(legIndex, legIndex), 'PIDF', 2*pi*bandwidthHz);
end
Kl = ss(blkdiag(legControllers{:}));

closedLoop = feedback(plant * Kl, eye(6));
closedLoopPoles = pole(closedLoop);
lowFrequencyRank = rank(evalfr(plant, 1e-3));
stable = all(real(closedLoopPoles) < -1e-7);

design = struct();
design.bandwidthHz = bandwidthHz;
design.plant = plant;
design.Kl = Kl;
design.closedLoop = closedLoop;
design.closedLoopPoles = closedLoopPoles;
design.lowFrequencyRank = lowFrequencyRank;
design.stable = stable;

if lowFrequencyRank ~= 6 || ~stable
    error('designSimscapeLengthController:DesignRejected', ...
        '长度控制对象低频秩为 %d，闭环稳定标志为 %d。', lowFrequencyRank, stable);
end
end
