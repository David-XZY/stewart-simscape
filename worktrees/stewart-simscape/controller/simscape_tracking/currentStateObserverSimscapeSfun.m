function currentStateObserverSimscapeSfun(block)
% currentStateObserverSimscapeSfun - Nonlinear 12-state observer runtime.
%
% Inputs: measured physical pose and one-sample-delayed QP force command.
% Outputs: estimated relative pose, physical pose, generalized velocity,
% and pose innovation. No measured velocity or applied-force truth enters
% the observer.

setup(block);
end

function setup(block)
block.NumDialogPrms = 1;
block.NumInputPorts = 2;
block.NumOutputPorts = 5;
for index = 1:2
    block.InputPort(index).Dimensions = 6;
    block.InputPort(index).DatatypeID = 0;
    block.InputPort(index).Complexity = 'Real';
    block.InputPort(index).DirectFeedthrough = true;
end
for index = 1:4
    block.OutputPort(index).Dimensions = 6;
    block.OutputPort(index).DatatypeID = 0;
    block.OutputPort(index).Complexity = 'Real';
end
block.OutputPort(5).Dimensions = 1;
block.OutputPort(5).DatatypeID = 0;
block.OutputPort(5).Complexity = 'Real';
runtime = block.DialogPrm(1).Data;
block.SampleTimes = [runtime.sampleTime, 0];
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('PostPropagationSetup', @postPropagationSetup);
block.RegBlockMethod('InitializeConditions', @initializeConditions);
block.RegBlockMethod('Outputs', @outputs);
end

function postPropagationSetup(block)
widths = [12, 6, 1, 1, 6, 1];
names = {'Estimate', 'Innovation', 'LastUpdateTime', 'Initialized', ...
    'LastMeasurement', 'ControllerStepTime'};
block.NumDworks = numel(widths);
for index = 1:numel(widths)
    block.Dwork(index).Name = names{index};
    block.Dwork(index).Dimensions = widths(index);
    block.Dwork(index).DatatypeID = 0;
    block.Dwork(index).Complexity = 'Real';
    block.Dwork(index).UsedAsDiscState = true;
end
end

function initializeConditions(block)
runtime = block.DialogPrm(1).Data;
block.Dwork(1).Data = [runtime.q0(:); runtime.initialQd(:)];
block.Dwork(2).Data = zeros(6, 1);
block.Dwork(3).Data = -inf;
block.Dwork(4).Data = 0;
block.Dwork(5).Data = runtime.q0(:);
block.Dwork(6).Data = 0;
end

function outputs(block)
runtime = block.DialogPrm(1).Data;
measuredPose = reshape(block.InputPort(1).Data, 6, 1);
previousForce = reshape(block.InputPort(2).Data, 6, 1);
currentTime = block.CurrentTime;
lastTime = block.Dwork(3).Data;
timeTolerance = max(1e-12, 32*eps(max(1, abs(currentTime))));

if block.Dwork(4).Data < 0.5
    estimate = [measuredPose; runtime.initialQd(:)];
    innovation = computePoseInnovation(measuredPose, estimate(1:6));
    block.Dwork(1).Data = estimate;
    block.Dwork(2).Data = innovation;
    block.Dwork(3).Data = currentTime;
    block.Dwork(4).Data = 1;
    block.Dwork(5).Data = measuredPose;
elseif abs(currentTime-lastTime) > timeTolerance
    dt = currentTime-lastTime;
    if ~(isfinite(dt) && dt > 0)
        error('currentStateObserverSimscapeSfun:InvalidUpdateInterval', ...
            'Observer update interval must be finite and positive.');
    end
    stepTimer = tic;
    [estimate, diagnostic] = stepCurrentStateLuenbergerObserver( ...
        block.Dwork(1).Data, measuredPose, previousForce, runtime.model, ...
        runtime.observer, dt, block.Dwork(5).Data);
    block.Dwork(6).Data = toc(stepTimer);
    block.Dwork(1).Data = estimate;
    block.Dwork(2).Data = diagnostic.innovation;
    block.Dwork(3).Data = currentTime;
    block.Dwork(5).Data = measuredPose;
end

estimate = block.Dwork(1).Data;
block.OutputPort(1).Data = estimate(1:6)-runtime.q0(:);
block.OutputPort(2).Data = estimate(1:6);
block.OutputPort(3).Data = estimate(7:12);
block.OutputPort(4).Data = block.Dwork(2).Data;
block.OutputPort(5).Data = block.Dwork(6).Data;
end
