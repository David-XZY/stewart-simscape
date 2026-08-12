function [Fcmd, diagnostic] = stepSafetyQpController(q, qd, qRef, qdRef, qddRef, Fprev, model, config, Fnom)
% stepSafetyQpController - 执行单个 SC-QP 控制周期
arguments
    q double
    qd double
    qRef double
    qdRef double
    qddRef double
    Fprev double
    model struct
    config struct
    Fnom double = []
end

if isempty(Fnom)
    Fnom = computeNominalForce(q, qd, qRef, qdRef, qddRef, model, config);
end
clf = buildTrackingClf(q, qd, qRef, qdRef, qddRef, model, config);
cbf = buildStewartCbfConstraints(q, qd, Fprev, model, config);
[Fcmd, diagnostic] = solveSafetyQpControl(Fnom, Fprev, clf, cbf, config);
diagnostic.nominalForce = Fnom(:);
end

function Fnom = computeNominalForce(q, qd, qRef, qdRef, qddRef, model, config)
switch config.nominalMode
    case "computed-torque"
        Fnom = computeComputedTorqueForce(qRef, qdRef, qddRef, q, qd, model, config);
    case "ihsid"
        Fnom = zeros(6, 1);
    case "pd"
        clf = buildTrackingClf(q, qd, qRef, qdRef, qddRef, model, config);
        jacobian = sgpJacobian(q, model);
        wrench = desiredWrenchFromAcceleration(clf.desiredAcceleration, model, config);
        if rcond(jacobian.Jv.') < model.num.rcondMin
            Fnom = pinv(jacobian.Jv.') * wrench;
        else
            Fnom = jacobian.Jv.' \ wrench;
        end
    otherwise
        error('stepSafetyQpController:UnsupportedNominalMode', ...
            '不支持的名义控制模式：%s。', config.nominalMode);
end
Fnom = min(max(Fnom(:), config.forceMin), config.forceMax);
end

function wrench = desiredWrenchFromAcceleration(qddDesired, model, config)
massMatrix = diag([ ...
    model.dynamics.totalMass * ones(1, 3), ...
    max(diag(model.dynamics.inertiaAtCOM_P))./max(config.characteristicLength, eps)^2 * ones(1, 3)]);
wrench = massMatrix * qddDesired(:);
end
