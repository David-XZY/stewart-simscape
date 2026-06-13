function simscapeData = buildSimscapeLengthControlData(model, scene, controlMode)
% buildSimscapeLengthControlData - 将 IHSID 参数映射为 Simscape 控制数据
%
% 说明：
%   几何、质量、惯量和负载质心严格读取当前优化模型与场景。优化模型忽略腿部
%   被动柔性，因此经典执行器的刚度和阻尼设为零。每段杆件保留 1e-3 kg，
%   仅用于避免 Simscape Multibody 退化质量分布，不代表真实杆件质量。

arguments
    model struct
    scene struct
    controlMode char {mustBeMember(controlMode, {'force', 'length-cascade', 'pose-length-cascade'})} = 'force'
end

regularizationMass = 1e-3;
platformHeight = 20e-3;

stewart = initializeStewartPlatform();
stewart = initializeFramesPositions(stewart, 'H', model.zHome, 'MO_B', 0);
stewart.platform_F.Fa = model.A;
stewart.platform_M.Mb = model.B(:, model.legMap);
stewart = computeJointsPose(stewart);
stewart = initializeStrutDynamics(stewart, 'K', zeros(6, 1), 'C', zeros(6, 1));
stewart = initializeJointDynamics(stewart, ...
    'type_F', 'universal_p', 'type_M', 'spherical_p');
stewart = initializeStewartPose(stewart);
stewart = initializeCylindricalPlatforms(stewart, ...
    'Fpr', model.rA, ...
    'Mpm', model.dynamics.platformMass, ...
    'Mph', platformHeight, ...
    'Mpr', model.rB);

% 优化模型采用薄圆盘移动平台，不使用显示厚度引入的额外惯量。
platformMass = model.dynamics.platformMass;
platformRadius = model.rB;
stewart.platform_M.I = diag([ ...
    0.25 * platformMass * platformRadius^2, ...
    0.25 * platformMass * platformRadius^2, ...
    0.50 * platformMass * platformRadius^2]);

stewart = initializeCylindricalStruts(stewart, ...
    'Fsm', regularizationMass, 'Msm', regularizationMass);
stewart = initializeInertialSensor(stewart, 'type', 'none');

objectMass = model.dynamics.objectMass;
objectRadius = scene.objectCylinder.radius;
objectLength = scene.objectCylinder.length;
payloadInertia = diag([ ...
    0.5 * objectMass * objectRadius^2, ...
    objectMass / 12 * (3 * objectRadius^2 + objectLength^2), ...
    objectMass / 12 * (3 * objectRadius^2 + objectLength^2)]);

payload = initializePayload( ...
    'type', 'rigid', ...
    'h', scene.objectCylinder.center_P(3), ...
    'm', objectMass, ...
    'I', payloadInertia);
ground = initializeGround('type', 'none');
if strcmp(controlMode, 'length-cascade')
    stewart.actuators.type = 5;
    controller = initializeController('type', 'length-cascade');
elseif strcmp(controlMode, 'pose-length-cascade')
    stewart.actuators.type = 5;
    controller = initializeController('type', 'pose-length-cascade');
else
    controller = initializeController('type', 'ref-track-L');
end
disturbances = initializeDisturbances();

mapping = computeMappingReport(stewart, payload, model);

simscapeData = struct();
simscapeData.stewart = stewart;
simscapeData.payload = payload;
simscapeData.ground = ground;
simscapeData.controller = controller;
simscapeData.controlMode = controlMode;
simscapeData.disturbances = disturbances;
simscapeData.gravity = model.g;
simscapeData.mapping = mapping;
simscapeData.approximations = struct( ...
    'actuatorStiffness', zeros(6, 1), ...
    'actuatorDamping', zeros(6, 1), ...
    'strutSegmentRegularizationMass', regularizationMass, ...
    'strutMassPurpose', '仅用于避免 Simscape Multibody 退化质量分布');
end

function mapping = computeMappingReport(stewart, payload, model)
% computeMappingReport - 验证 Simscape 刚体组合与优化模型一致
platformMass = stewart.platform_M.M;
payloadMass = payload.m;
payloadPosition = [0; 0; payload.h];
totalMass = platformMass + payloadMass;
combinedCom = payloadMass * payloadPosition / totalMass;

platformShift = parallelAxis(platformMass, -combinedCom);
payloadShift = parallelAxis(payloadMass, payloadPosition - combinedCom);
combinedInertia = stewart.platform_M.I + platformShift + payload.I + payloadShift;

mapping = struct();
mapping.totalMass = totalMass;
mapping.comP = combinedCom;
mapping.inertiaAtCOM_P = combinedInertia;
mapping.totalMassError = abs(totalMass - model.dynamics.totalMass);
mapping.comError = max(abs(combinedCom - model.dynamics.comP));
mapping.inertiaError = max(abs(combinedInertia - model.dynamics.inertiaAtCOM_P), [], 'all');
mapping.initialLengthError = max(abs(stewart.geometry.l - sgpIK(model.qHome, model).L));

tolerance = 1e-10;
if max([mapping.totalMassError, mapping.comError, mapping.inertiaError, ...
        mapping.initialLengthError]) > tolerance
    error('buildSimscapeLengthControlData:MappingMismatch', ...
        '优化模型到 Simscape 的参数映射误差超过 %.3e。', tolerance);
end
end

function inertiaShift = parallelAxis(massValue, offset)
% parallelAxis - 平行轴定理附加惯量
inertiaShift = massValue * ((offset.' * offset) * eye(3) - offset * offset.');
end
