function layout = makeCandidateControllerDiagnosticLayout()
% makeCandidateControllerDiagnosticLayout - Fixed Simulink logging contract.
layout = struct();
layout.controllerStepTime = 1;
layout.qpSolveTime = 2;
layout.feasible = 3;
layout.usedFallback = 4;
layout.minimumCbfResidual = 5;
layout.dobWrenchEstimate = 6:11;
layout.dobLegCompensation = 12:17;
layout.qpAwareAntiWindupMismatch = 18:23;
layout.governedAttitudePerturbation = 24:26;
layout.governedAttitudeRate = 27:29;
layout.governedAttitudeAcceleration = 30:32;
layout.width = 32;
end
