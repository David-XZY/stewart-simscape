function [s,replanned,entry]=ch5BaselineReplanV90(s,q,qd,goal,o,r,time)
% Keep a verified retreat while the observed goal hold remains obstructed.
% Revalidate every 0.2 s; resume IHSID goal planning when this gate clears.
sameGoal=norm(goal-s.goal)<=1e-10;
backup=s.plan.accepted&&isfield(s.plan,'reference_mode')&&strcmp(s.plan.reference_mode,'temporary_initial_safe_pose');
if sameGoal&&backup
    hold=r.c.H*r.c.Ts;
    stationary=struct('accepted',true,'t',[0,hold],'q',repmat(goal,1,2),'qd',zeros(6,2));
    goalCheck=ch5SpeedPlanPostcheck(stationary,o,r);
    if ~goalCheck.accepted
        temporary=s;temporary.goal=s.plan.q(:,end);temporary.plan.reference_mode='task';
        [retained,entry]=ch5BaselineRetainV83(temporary,q,qd,temporary.goal,o,r,time);
        if retained
            s.attemptTime=time;replanned=false;
            entry.reference_decision='retained_checked_retreat_while_goal_hold_obstructed';
            entry.goal_hold_check=goalCheck;entry.desired_goal=goal;entry.temporary_goal=temporary.goal;
            return;
        end
    end
end
[s,replanned,entry]=ch5BaselineReplanV83(s,q,qd,goal,o,r,time);
end
