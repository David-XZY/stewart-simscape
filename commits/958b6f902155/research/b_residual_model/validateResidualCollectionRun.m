function report=validateResidualCollectionRun(result,model,tolerance)
% Identification uses actual force, including runs with imperfect feedback.
% Reject contacts, travel violations, force-range violations and singular states.
if nargin<3, tolerance=1e-6; end
m=result.metrics;
report=struct('passed',m.collisionViolationCount==0 && m.lengthViolationCount==0 && ...
    m.forceMargin>=-tolerance && m.minSigma>=model.singularity.sigmaMinSafe-tolerance, ...
    'engineeringPassed',m.engineeringPassed,'budgetPassed',m.budgetPassed);
assert(report.passed,'ABFD:ResidualRunRejected', ...
    'Actual plant left the collision-free, in-range, nonsingular identification domain.');
end
