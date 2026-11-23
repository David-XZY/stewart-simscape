function [radius,accept]=updateCodesignTrustRegion(radius,oldCost,newCost,passed,oldViolation,newViolation,tolerance)
if nargin<5, oldViolation=0; end
if nargin<6, newViolation=0; end
if nargin<7, tolerance=1e-6; end
% Feasibility takes priority while entering the tightened feasible set.
accept=passed && isfinite(newCost) && isfinite(newViolation) && ...
    ((oldViolation>tolerance && newViolation<oldViolation) || ...
    (oldViolation<=tolerance && newViolation<=tolerance && newCost<oldCost));
if accept, radius=min(0.5,radius*1.5); else, radius=max(1e-4,radius/2); end
end
