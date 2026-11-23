function [accepted,r]=bdAccept(bundle,task,policy,clockHandle,budget,c,solver)
if nargin<8, solver=[]; end
timer=tic;
if isempty(solver), physical=bdValidate(bundle,task,c,clockHandle,budget); else, physical=solver.report; end
ps=toc(timer);
r=struct('physical',physical,'physicalPassed',physical.passed,'qualityPassed',false, ...
    'quality',[],'physicalSeconds',ps,'qualitySeconds',0,'reason',physical.reason);
eligible=physical.passed;
if strcmp(bundle.kind,'ihsid')
    assert(~isempty(solver),'BD:SolverEvidence','IHSID requires solver evidence.');
    eligible=eligible && c3SolverGate(solver.solverSuccess,solver.solverStatus,solver.maxEquality,solver.maxInequality,solver.maxBounds,solver.finite,toc(clockHandle)<=budget,physical.passed);
    if ~eligible && physical.passed, r.reason='solver_gate'; end
end
if eligible && strcmp(policy,'quality')
    assert(c.qualityFrozen,'BD:QualityNotFrozen','Quality thresholds must be frozen.');
    timer=tic; r.quality=bdQuality(bundle,task,c); r.qualitySeconds=toc(timer);
    r.qualityPassed=r.quality.positionDetour<=c.positionCap && r.quality.rotationDetour<=c.rotationCap;
    eligible=r.qualityPassed; if ~eligible, r.reason='quality_below_standard'; end
end
accepted=eligible && toc(clockHandle)<=budget;
if toc(clockHandle)>budget, r.reason='timeout'; end
end
