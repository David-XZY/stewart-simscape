function measuredStateAdapterSimscapeSfun(block)
% measuredStateAdapterSimscapeSfun - Convert pose/velocity measurements to x.
%
% Inputs are the measured relative pose and measured spatial velocity
% [world linear velocity; world angular velocity]. Outputs are physical
% q and generalized qdot. This block is the only direct-state adapter used
% by the runtime controller and by offline truth-only evaluation logging.

setup(block);
end

function setup(block)
block.NumDialogPrms = 1;
block.NumInputPorts = 2;
block.NumOutputPorts = 2;
for index = 1:2
    block.InputPort(index).Dimensions = 6;
    block.InputPort(index).DatatypeID = 0;
    block.InputPort(index).Complexity = 'Real';
    block.InputPort(index).DirectFeedthrough = true;
    block.OutputPort(index).Dimensions = 6;
    block.OutputPort(index).DatatypeID = 0;
    block.OutputPort(index).Complexity = 'Real';
end
runtime = block.DialogPrm(1).Data;
block.SampleTimes = [runtime.sampleTime, 0];
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('Outputs', @outputs);
end

function outputs(block)
runtime = block.DialogPrm(1).Data;
relativePose = reshape(block.InputPort(1).Data, 6, 1);
spatialVelocity = reshape(block.InputPort(2).Data, 6, 1);
absolutePose = runtime.q0(:)+relativePose;
rateMap = rpyRateMapZYX(absolutePose(4:6));
if rcond(rateMap) < runtime.rpyRateRcondMin
    rpyRate = pinv(rateMap)*spatialVelocity(4:6);
else
    rpyRate = rateMap\spatialVelocity(4:6);
end
generalizedVelocity = [spatialVelocity(1:3); rpyRate];
if any(~isfinite([absolutePose; generalizedVelocity]))
    error('measuredStateAdapterSimscapeSfun:NonfiniteMeasurement', ...
        'The direct Simscape state measurement is nonfinite.');
end
block.OutputPort(1).Data = absolutePose;
block.OutputPort(2).Data = generalizedVelocity;
end
