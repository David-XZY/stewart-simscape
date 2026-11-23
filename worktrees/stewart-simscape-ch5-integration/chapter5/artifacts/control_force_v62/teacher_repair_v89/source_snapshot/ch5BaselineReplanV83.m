function [s,replanned,entry]=ch5BaselineReplanV83(s,q,qd,goal,o,r,time)
% A newly unsafe goal hold invalidates the old arrival budget as well as reuse.
% Keep IHSID, four starts, LQI continuity and all original physical checks.
[retained,precheck]=ch5BaselineRetainV83(s,q,qd,goal,o,r,time);
if retained,s.attemptTime=time;replanned=false;entry=precheck;return;end
invalidated=~isempty(precheck.previous_reference_check)&&~precheck.previous_reference_check.accepted;
oldArrival=s.arrivalTime;
if invalidated,s.arrivalTime=min(time+6,r.c.episode_seconds-r.c.arrival_hold_seconds);end
% The rejection path in V67 also rechecks an old reference. Apply the same
% horizon there; the caller's periodic replanning interval remains 0.2 s.
checkedRuntime=r;checkedRuntime.c.replan_seconds=max(r.c.replan_seconds,r.c.H*r.c.Ts);
[s,replanned,entry]=ch5BaselineReplanV67(s,q,qd,goal,o,checkedRuntime,time);
entry.horizon_reuse_check=precheck;entry.arrival_rescheduled_for_obstacle=invalidated;
entry.previous_arrival_time=oldArrival;entry.arrival_time=s.arrivalTime;
end
