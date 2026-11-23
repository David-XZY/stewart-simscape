function result=finalizeAbfdCodesignAcceptance(result,initialFeasibility,feasibility,history,physicalFallback)
% An unchanged feasible sequential seed is a fallback, not successful A work.
accepted=sum([history.accepted]); passed=result.passed && feasibility.passed && accepted>0;
result.codesign=struct('initialFeasibility',initialFeasibility,'finalFeasibility',feasibility, ...
    'acceptedIterations',accepted,'pairPassed',feasibility.passed,'passed',passed);
result.passed=passed;
if ~passed
    result.failureReason='No refined trajectory/gain pair passed the tightened constraints.';
    if feasibility.passed && accepted==0
        result.failureReason='Only the original feasible pair remains; no A refinement was accepted.';
    end
    result.lastFeasible=physicalFallback;
end
end
