function clf = buildTrackingClf(q, qd, qRef, qdRef, qddRef, model, config)
% buildTrackingClf - 构造关于支链力的近似线性 CLF 跟踪约束
arguments
    q double
    qd double
    qRef double
    qdRef double
    qddRef double
    model struct
    config struct
end

q = q(:);
qd = qd(:);
qRef = qRef(:);
qdRef = qdRef(:);
qddRef = qddRef(:);
validateattributes(q, {'double'}, {'numel', 6, 'finite'});
validateattributes(qd, {'double'}, {'numel', 6, 'finite'});
validateattributes(qRef, {'double'}, {'numel', 6, 'finite'});
validateattributes(qdRef, {'double'}, {'numel', 6, 'finite'});
validateattributes(qddRef, {'double'}, {'numel', 6, 'finite'});

e = q - qRef;
ed = qd - qdRef;
z = [e; ed];
P = config.clf.P;
V = z.' * P * z;

Kp = config.clf.positionGain;
Kd = config.clf.velocityGain;
desiredAcceleration = qddRef - Kd * ed - Kp * e;
accelMap = platformAccelerationMap(q, model, config);

% 由二阶收敛条件构造标量 CLF：ed' * (qdd - qdd_des) <= -c V。
A = ed.' * accelMap;
b = ed.' * desiredAcceleration - config.clf.rate * max(V, 0);
if norm(ed) < 1e-10
    A = zeros(1, 6);
    b = config.clf.rate * max(V, 0);
end

clf = struct();
clf.A = A;
clf.b = b;
clf.V = V;
clf.error = e;
clf.errorRate = ed;
clf.desiredAcceleration = desiredAcceleration;
clf.VdotNominal = A * zeros(6, 1) - ed.' * desiredAcceleration;
clf.accelMap = accelMap;
end

function accelMap = platformAccelerationMap(q, model, config)
jacobian = sgpJacobian(q, model);
massMatrix = diag([ ...
    model.dynamics.totalMass * ones(1, 3), ...
    max(diag(model.dynamics.inertiaAtCOM_P))./max(config.characteristicLength, eps)^2 * ones(1, 3)]);
accelMap = massMatrix \ jacobian.Jv.';
end
