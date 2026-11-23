function [accepted,r]=pgAccept(bundle,task,policy,clock,budget,c,solver)
if nargin<7, solver=[]; end
timer=tic;
if isempty(solver), physical=pgValidate(bundle,task,c,clock,budget); else, physical=solver.report; end
r=struct('physical',physical,'physicalPassed',physical.passed,'qualityPassed',false,'quality',[], ...
    'physicalSeconds',toc(timer),'qualitySeconds',0,'reason',physical.reason);
eligible=physical.passed;
if strcmp(bundle.kind,'ihsid')
    assert(~isempty(solver),'PG:SolverEvidence','Current-task IHSID requires current-task solver evidence.');
    eligible=eligible&&c3SolverGate(solver.solverSuccess,solver.solverStatus,solver.maxEquality,solver.maxInequality,solver.maxBounds,solver.finite,toc(clock)<=budget,physical.passed);
    if ~eligible&&physical.passed, r.reason='solver_gate'; end
end
if eligible&&strcmp(policy,'quality')
    assert(c.qualityFrozen,'PG:Quality','Shape-specific quality caps not frozen.'); timer=tic;
    r.quality=pgQuality(bundle,task,c); r.qualitySeconds=toc(timer);
    r.qualityPassed=r.quality.positionDetour<=c.positionCap&&r.quality.rotationDetour<=c.rotationCap;
    eligible=r.qualityPassed; if ~eligible, r.reason='quality_below_standard'; end
end
accepted=eligible&&toc(clock)<=budget; if toc(clock)>budget, r.reason='timeout'; end
end
