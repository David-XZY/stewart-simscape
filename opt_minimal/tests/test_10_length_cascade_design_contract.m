function test_10_length_cascade_design_contract
% test_10_length_cascade_design_contract - 验证长度级联逐环自动整定
optRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

model = buildOptModelCustom();
config = makeSimscapeLengthCascadeConfig(model);
design = designSimscapeLengthCascadeController(config);

assert(design.stable);
assert(abs(design.sampleTime - 0.01) < eps);
assert(abs(design.innerBandwidthHz - 10) < eps);
assert(abs(design.outerBandwidthHz - 2) < eps);
assert(design.positionGainScale == 1);
assert(design.velocityGainScale == 0.7);
assert(isequal(size(design.Kpos), [6, 1]));
assert(numel(design.velocityControllers) == 6);
assert(all(design.Kpos > 0));
assert(all(abs(design.velocityClosedLoopPoles) < 1, 'all'));
assert(all(abs(design.positionClosedLoopPoles) < 1, 'all'));
assert(isequal(config.speedLimit, model.actuator.ldotMax));
assert(isequal(config.accelerationLimit, model.actuator.lddotMax));
assert(~config.gravityEnabled);

scaledConfig = makeSimscapeLengthCascadeConfig(model, struct( ...
    'positionGainScale', 0.7, 'velocityGainScale', 0.5));
scaledDesign = designSimscapeLengthCascadeController(scaledConfig);
assert(scaledDesign.stable);
assert(all(scaledDesign.Kpos < design.Kpos));
assert(all(scaledDesign.velocityKp < design.velocityKp));
assert(all(scaledDesign.velocityKi < design.velocityKi));
assert(all(scaledDesign.velocityKd < design.velocityKd));
end
