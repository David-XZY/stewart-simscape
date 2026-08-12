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
layout.width = 17;
end
