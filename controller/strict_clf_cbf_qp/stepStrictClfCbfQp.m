function [forceCommand, diagnostic] = stepStrictClfCbfQp( ...
    q, qd, qRef, qdRef, qddRef, nominalForce, previousForce, model, scene, ...
    config, knownForceOffset, externalWrenchEstimate)
% stepStrictClfCbfQp - Execute one strict CLF-CBF-QP control interval.
%
% Inputs follow q=[position; ZYX roll-pitch-yaw] and F is the six axial-leg
% force vector. nominalForce is the component optimized by the QP.  An
% optional knownForceOffset (for example the current DOB compensation) is
% included in every acceleration-level CLF/CBF prediction and added to the
% optimized component before the total leg-force command is returned. Its
% paired externalWrenchEstimate is included so that CBFs predict only the
% uncompensated DOB residual rather than double-counting cancellation force.
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
    knownForceOffset double = zeros(6, 1)
    externalWrenchEstimate double = zeros(6, 1)
end

q = reshape(q, 6, 1);
qd = reshape(qd, 6, 1);
qRef = reshape(qRef, 6, 1);
qdRef = reshape(qdRef, 6, 1);
qddRef = reshape(qddRef, 6, 1);
nominalForce = reshape(nominalForce, 6, 1);
previousForce = reshape(previousForce, 6, 1);
knownForceOffset = reshape(knownForceOffset, 6, 1);
externalWrenchEstimate = reshape(externalWrenchEstimate, 6, 1);

baseEvaluation = evaluateStrictClfCbfAdModel(q, qd, model, scene, config);
evaluation = applyKnownForceOffsetToStrictEvaluation( ...
    baseEvaluation, knownForceOffset, externalWrenchEstimate);
[clf, cbf] = buildStrictClfCbfConstraints(q, qd, qRef, qdRef, qddRef, ...
    model, scene, config, evaluation);
[forceCommand, diagnostic] = solveStrictClfCbfQp( ...
    nominalForce, previousForce, clf, cbf, config, knownForceOffset);

diagnostic.nominalForce = nominalForce+knownForceOffset;
diagnostic.regulatedNominalForce = nominalForce;
diagnostic.previousForce = previousForce;
diagnostic.knownForceOffset = knownForceOffset;
diagnostic.externalWrenchEstimate = externalWrenchEstimate;
diagnostic.clf = clf;
diagnostic.cbf = cbf;
diagnostic.affineDynamics = struct('drift', evaluation.drift, ...
    'inputMap', evaluation.inputMap, 'massMatrix', evaluation.H, ...
    'biasWrench', evaluation.rigidBias, ...
    'unshiftedDrift', baseEvaluation.drift, ...
    'knownActuatorAcceleration', evaluation.knownActuatorAcceleration, ...
    'knownExternalAcceleration', evaluation.knownExternalAcceleration, ...
    'knownGeneralizedAcceleration', evaluation.knownGeneralizedAcceleration, ...
    'knownLegAcceleration', evaluation.knownLegAcceleration);
diagnostic.cbfValues = cbf.stateMargins;
diagnostic.cbfNames = cbf.stateNames;
diagnostic.derivativeBackend = evaluation.derivativeBackend;
diagnostic.usesFiniteDifferences = evaluation.usesFiniteDifferences;
diagnostic.sigmaLowerSquared = evaluation.sigmaLowerSquared;
diagnostic.collisionCertificates = evaluation.collisionGap;
end
