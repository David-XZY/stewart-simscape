function adjusted = applyKnownForceOffsetToStrictEvaluation( ...
        evaluation, forceOffset, externalWrenchEstimate)
% applyKnownForceOffsetToStrictEvaluation - Shift affine drift by known force.
%
% The strict QP then chooses a regulated leg-force component Freg while the
% plant receives Ftotal=Freg+Foffset and is subject to an estimated external
% generalized wrench. All relative-degree-two constraints use the net known
% generalized and leg-acceleration contributions. A correctly mapped DOB
% force and its disturbance estimate therefore cancel in the prediction.
arguments
    evaluation struct
    forceOffset double
    externalWrenchEstimate double = zeros(6, 1)
end

forceOffset = reshape(forceOffset, 6, 1);
externalWrenchEstimate = reshape(externalWrenchEstimate, 6, 1);
validateattributes(forceOffset, {'double'}, ...
    {'real', 'finite', 'size', [6, 1]}, mfilename, 'forceOffset');
validateattributes(externalWrenchEstimate, {'double'}, ...
    {'real', 'finite', 'size', [6, 1]}, mfilename, ...
    'externalWrenchEstimate');

requiredFields = {'drift', 'inputMap', 'legAccelerationDrift', ...
    'legAccelerationMap', 'H', 'Jq'};
for index = 1:numel(requiredFields)
    if ~isfield(evaluation, requiredFields{index})
        error('applyKnownForceOffsetToStrictEvaluation:MissingField', ...
            'Evaluation is missing field %s.', requiredFields{index});
    end
end

adjusted = evaluation;
adjusted.unshiftedDrift = evaluation.drift;
adjusted.unshiftedLegAccelerationDrift = evaluation.legAccelerationDrift;
adjusted.knownForceOffset = forceOffset;
adjusted.externalWrenchEstimate = externalWrenchEstimate;
adjusted.knownActuatorAcceleration = evaluation.inputMap*forceOffset;
adjusted.knownExternalAcceleration = evaluation.H\externalWrenchEstimate;
adjusted.knownGeneralizedAcceleration = adjusted.knownActuatorAcceleration+ ...
    adjusted.knownExternalAcceleration;
adjusted.knownLegAcceleration = evaluation.legAccelerationMap*forceOffset+ ...
    evaluation.Jq*adjusted.knownExternalAcceleration;
adjusted.drift = evaluation.drift+adjusted.knownGeneralizedAcceleration;
adjusted.legAccelerationDrift = evaluation.legAccelerationDrift+ ...
    adjusted.knownLegAcceleration;
end
