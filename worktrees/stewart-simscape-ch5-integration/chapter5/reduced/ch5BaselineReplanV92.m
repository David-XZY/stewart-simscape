function [s,replanned,entry]=ch5BaselineReplanV92(s,q,qd,goal,o,r,time)
% Preserve the original arrival-budget renewal event during retreat reuse.
% A blocked present goal need not block a trajectory arriving after clearance.
if s.arrivalTime-time<.5
    [s,replanned,entry]=ch5BaselineReplanV83(s,q,qd,goal,o,r,time);
    entry.arrival_budget_event_preserved=true;
else
    [s,replanned,entry]=ch5BaselineReplanV90(s,q,qd,goal,o,r,time);
    entry.arrival_budget_event_preserved=false;
end
end
