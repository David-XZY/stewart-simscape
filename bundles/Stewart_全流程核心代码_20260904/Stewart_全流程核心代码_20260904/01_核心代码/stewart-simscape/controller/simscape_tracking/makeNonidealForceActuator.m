function actuator = makeNonidealForceActuator(model, config)
% makeNonidealForceActuator - 构造非理想力执行器参数
arguments
    model struct
    config struct = struct()
end

forceLimit = max(abs([model.actuator.forceMin(:), model.actuator.forceMax(:)]), [], 2);
actuator = struct();
actuator.type = 'nonideal-force-actuator';
actuator.forceLimit = forceLimit;
actuator.forceRateLimit = 4 * forceLimit / 0.01;
actuator.timeConstant = 0.001;
actuator.deadzone = zeros(6, 1);
actuator.inputDelay = 1e-6;
actuator.initialForce = zeros(6, 1);

if isfield(config, 'forceLimit')
    actuator.forceLimit = config.forceLimit * ones(6, 1);
end
if isfield(config, 'forceRateLimit')
    actuator.forceRateLimit = config.forceRateLimit * ones(6, 1);
end
if isfield(config, 'forceLagTimeConstant')
    actuator.timeConstant = config.forceLagTimeConstant;
end
if isfield(config, 'forceDeadzone')
    actuator.deadzone = config.forceDeadzone(:);
end
if isfield(config, 'forceInputDelay')
    actuator.inputDelay = config.forceInputDelay;
end
if isfield(config, 'initialForce')
    actuator.initialForce = config.initialForce(:);
end

validateattributes(actuator.forceLimit, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(actuator.forceRateLimit, {'double'}, {'size', [6, 1], 'positive', 'finite'});
validateattributes(actuator.timeConstant, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(actuator.deadzone, {'double'}, {'size', [6, 1], 'nonnegative', 'finite'});
validateattributes(actuator.inputDelay, {'double'}, {'scalar', 'nonnegative', 'finite'});
validateattributes(actuator.initialForce, {'double'}, {'size', [6, 1], 'finite'});
end
