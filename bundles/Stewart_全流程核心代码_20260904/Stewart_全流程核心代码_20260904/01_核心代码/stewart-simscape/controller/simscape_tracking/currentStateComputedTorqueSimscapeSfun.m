function currentStateComputedTorqueSimscapeSfun(block)
% currentStateComputedTorqueSimscapeSfun - Runtime current-state feedforward.
%
% Inputs are qhat, qdhat, and reference qdd. The output contains computed
% torque only; the existing LQI correction remains a separate summand.

setup(block);
end

function setup(block)
block.NumDialogPrms = 1;
block.NumInputPorts = 3;
block.NumOutputPorts = 2;
for index = 1:3
    block.InputPort(index).Dimensions = 6;
    block.InputPort(index).DatatypeID = 0;
    block.InputPort(index).Complexity = 'Real';
    block.InputPort(index).DirectFeedthrough = true;
end
block.OutputPort(1).Dimensions = 6;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';
block.OutputPort(2).Dimensions = 1;
block.OutputPort(2).DatatypeID = 0;
block.OutputPort(2).Complexity = 'Real';
runtime = block.DialogPrm(1).Data;
block.SampleTimes = [runtime.sampleTime, 0];
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('Outputs', @outputs);
end

function outputs(block)
runtime = block.DialogPrm(1).Data;
estimatedPose = reshape(block.InputPort(1).Data, 6, 1);
estimatedVelocity = reshape(block.InputPort(2).Data, 6, 1);
referenceAcceleration = reshape(block.InputPort(3).Data, 6, 1);
stepTimer = tic;
force = computeComputedTorqueForce(estimatedPose, estimatedVelocity, ...
    referenceAcceleration, estimatedPose, estimatedVelocity, runtime.model, ...
    runtime.currentStateComputedTorqueConfig);
stepTime = toc(stepTimer);
if any(~isfinite(force))
    error('currentStateComputedTorqueSimscapeSfun:NonfiniteForce', ...
        'Current-state computed torque is nonfinite.');
end
block.OutputPort(1).Data = force;
block.OutputPort(2).Data = stepTime;
end
