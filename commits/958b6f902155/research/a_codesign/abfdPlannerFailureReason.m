function reason=abfdPlannerFailureReason(result)
% Distinguish optimizer convergence from independent trajectory acceptance.
if result.passed, reason='accepted'; return; end
if isfield(result,'denseReport')
    r=result.denseReport;
    reason=sprintf('Dense reference rejected: violation=%g at t=%g (constraint %d), boundary=%g, insertion=%g, dynamics=%g; solver=%s', ...
        r.maxViolation,r.worstTime,r.worstConstraintIndex,r.maxBoundaryError, ...
        r.maxInsertionError,r.maxDynamicsResidual,result.solverStatus);
else
    reason=sprintf('NLP rejected: solver=%s, equality=%g, inequality=%g', ...
        result.solverStatus,result.maxEq,result.maxIneq);
end
end
