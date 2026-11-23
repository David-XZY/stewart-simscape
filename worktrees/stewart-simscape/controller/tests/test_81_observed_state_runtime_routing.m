function test_81_observed_state_runtime_routing
% Candidate CT, LQI, and QP state inputs must all trace to the observer.
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'strict_clf_cbf_qp'));
addpath(fullfile(controllerRoot, 'lqi_schedule_comparison'));

modelFile = which('stewart_platform_model.slx');
before = dir(modelFile);
setup = prepareSimscapeStrictClfCbfQp('', ...
    struct('stateFeedbackMode', 'observed-current-state'));
cleanup = onCleanup(@() closeWithoutSaving(setup.modelName));
modelName = setup.modelName;
observer = [modelName, '/Current State Observer'];
adapter = [modelName, '/Strict Measured State Adapter'];
delay = [modelName, '/Previous QP Force'];
torque = [modelName, '/Current State Computed Torque'];
qp = [modelName, '/Strict CLF-CBF-QP Filter'];

qpPorts = get_param(qp, 'PortHandles');
assert(sourceBlock(qpPorts.Inport(1)) == string(observer));
assert(sourceBlock(qpPorts.Inport(2)) == string(observer));
sumPorts = get_param([modelName, '/Sum Feedback Feedforward'], 'PortHandles');
assert(sourceBlock(sumPorts.Inport(2)) == string(torque));
busPorts = get_param([modelName, '/Bus Creator'], 'PortHandles');
assert(sourceBlock(busPorts.Inport(1)) == string(observer));

observerPorts = get_param(observer, 'PortHandles');
assert(sourceBlock(observerPorts.Inport(1)) == string(adapter));
assert(sourceBlock(observerPorts.Inport(2)) == string(delay));
delayPorts = get_param(delay, 'PortHandles');
assert(sourceBlock(delayPorts.Inport(1)) == string(qp));
assert(~setup.strictRuntime.observer.usesVelocityMeasurement);
assert(~setup.strictRuntime.observer.usesAppliedForceTruth);
assert(setup.strictInterface.modelSaved == false);

clear cleanup;
after = dir(modelFile);
assert(before.bytes == after.bytes && before.datenum == after.datenum, ...
    'Official SLX changed during runtime installation.');
end

function name = sourceBlock(destinationPort)
line = get_param(destinationPort, 'Line');
assert(line ~= -1, 'Expected input port to be connected.');
source = get_param(line, 'SrcBlockHandle');
name = string(getfullname(source));
end

function closeWithoutSaving(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
