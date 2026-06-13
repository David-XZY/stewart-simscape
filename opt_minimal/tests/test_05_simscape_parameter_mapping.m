function test_05_simscape_parameter_mapping
% test_05_simscape_parameter_mapping - 验证优化模型到 Simscape 的参数映射
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
simscapeData = buildSimscapeLengthControlData(model, scene);
legacyPayload = initializePayload('type', 'rigid', 'h', 0.25);
assert(max(abs(legacyPayload.center - [0; 0; 0.25])) < 1e-12);
assert(abs(legacyPayload.length - 0.50) < 1e-12);

homeKin = sgpIK(model.qHome, model);
assert(max(abs(simscapeData.stewart.geometry.l - homeKin.L)) < 1e-12);
assert(simscapeData.mapping.totalMassError < 1e-12);
assert(simscapeData.mapping.comError < 1e-12);
assert(simscapeData.mapping.inertiaError < 1e-12);
assert(max(abs(simscapeData.payload.center - scene.objectCylinder.center_P)) < 1e-12);
assert(max(abs(simscapeData.payload.axis - scene.objectCylinder.axis_P)) < 1e-12);
assert(abs(simscapeData.payload.radius - scene.objectCylinder.radius) < 1e-12);
assert(abs(simscapeData.payload.length - scene.objectCylinder.length) < 1e-12);
assert(max(abs(simscapeData.payload.R * [0; 0; 1] - scene.objectCylinder.axis_P)) < 1e-12);
assert(max(abs(simscapeData.payload.R * simscapeData.payload.I_local * ...
    simscapeData.payload.R.' - simscapeData.payload.I), [], 'all') < 1e-12);
assert(all(simscapeData.stewart.actuators.K == 0));
assert(all(simscapeData.stewart.actuators.C == 0));
assert(all(simscapeData.stewart.struts_F.M == 1e-3));
assert(all(simscapeData.stewart.struts_M.M == 1e-3));
assert(simscapeData.controller.type == 5);
assert(isequal(simscapeData.gravity, model.g));
end
