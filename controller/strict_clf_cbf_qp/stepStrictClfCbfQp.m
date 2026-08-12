function [forceCommand, diagnostic] = stepStrictClfCbfQp( ...
    q, qd, qRef, qdRef, qddRef, nominalForce, previousForce, model, scene, config)
% stepStrictClfCbfQp - Execute one strict CLF-CBF-QP control interval.
%
% Inputs follow q=[position; ZYX roll-pitch-yaw] and F is the six axial-leg
% force vector.  nominalForce is the complete computed-torque + LQI command.
arguments
    q double
    qd double
    qRef double
    qdRef double
    qddRef double
    nominalForce double
    previousForce double
    model struct
    scene struct
    config struct
end

q = reshape(q, 6, 1);
qd = reshape(qd, 6, 1);
qRef = reshape(qRef, 6, 1);
qdRef = reshape(qdRef, 6, 1);
qddRef = reshape(qddRef, 6, 1);
nominalForce = reshape(nominalForce, 6, 1);
previousForce = reshape(previousForce, 6, 1);

evaluation = evaluateStrictClfCbfAdModel(q, qd, model, scene, config);
[clf, cbf] = buildStrictClfCbfConstraints(q, qd, qRef, qdRef, qddRef, ...
    model, scene, config, evaluation);
[forceCommand, diagnostic] = solveStrictClfCbfQp( ...
    nominalForce, previousForce, clf, cbf, config);

diagnostic.nominalForce = nominalForce;
diagnostic.previousForce = previousForce;
diagnostic.clf = clf;
diagnostic.cbf = cbf;
diagnostic.affineDynamics = struct('drift', evaluation.drift, ...
    'inputMap', evaluation.inputMap, 'massMatrix', evaluation.H, ...
    'biasWrench', evaluation.rigidBias);
diagnostic.cbfValues = cbf.stateMargins;
diagnostic.cbfNames = cbf.stateNames;
diagnostic.derivativeBackend = evaluation.derivativeBackend;
diagnostic.usesFiniteDifferences = evaluation.usesFiniteDifferences;
diagnostic.sigmaLowerSquared = evaluation.sigmaLowerSquared;
diagnostic.collisionCertificates = evaluation.collisionGap;
end
