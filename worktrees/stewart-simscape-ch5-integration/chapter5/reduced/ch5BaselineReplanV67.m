function [s,replanned,entry]=ch5BaselineReplanV67(s,q,qd,goal,o,r,time)
% Rejected replanning never licenses a stale reference under new obstacles.
deadline=r.c.episode_seconds-r.c.arrival_hold_seconds;
if norm(goal-s.goal)>1e-10||s.arrivalTime-time<.5,s.arrivalTime=min(time+6,deadline);end
duration=s.arrivalTime-time;replanned=false;
if duration>=.05,primary=ch5SpeedIhsidV66(q,qd,goal,o,r,duration,time);
else,primary=struct('accepted',false,'attempts',{{}},'status','arrival_budget_exhausted','seconds',0);end
s.attemptTime=time;entry=struct('time',time,'observed',o,'plan',primary,'reference_decision','none','previous_reference_check',[],'backup_plan',[]);
if primary.accepted
    primary.reference_mode='task';s.plan=primary;s.planTime=time;replanned=true;entry.reference_decision='new_task_plan';return;
end
if s.plan.accepted
    remaining=remainingPlan(s.plan,max(0,time-s.planTime),r.c.replan_seconds);
    checked=ch5SpeedPlanPostcheck(remaining,o,r);entry.previous_reference_check=checked;
    if checked.accepted,entry.reference_decision='retained_after_updated_obstacle_check';return;end
end
backupDuration=min(2,r.c.episode_seconds-time);
if backupDuration>=.05
    backup=ch5SpeedIhsidV66(q,qd,s.backup_pose,o,r,backupDuration,time);
else,backup=struct('accepted',false,'attempts',{{}},'status','backup_budget_exhausted','seconds',0);end
entry.backup_plan=backup;
if backup.accepted
    backup.reference_mode='temporary_initial_safe_pose';s.plan=backup;s.planTime=time;replanned=true;
    entry.reference_decision='new_backup_plan';
else
    % An unverified braking reference is explicitly marked unaccepted.
    % QP/hold failures and execution of this reference have no safety claim.
    T=max(.1,min(.5,r.c.episode_seconds-time));s.plan=struct('accepted',false,'t',[0,T], ...
        'q',[q,q+.5*T*qd],'qd',[qd,zeros(6,1)],'reference_mode','unverified_braking');
    s.planTime=time;entry.reference_decision='unverified_braking_after_both_rejections';
end
end
function remaining=remainingPlan(plan,age,hold)
times=sort([0,plan.t(plan.t>age+1e-9)-age,max(hold,plan.t(end)-age)]);
times=times([true,diff(times)>1e-9]);
Q=zeros(6,numel(times));V=Q;
for i=1:numel(times),[Q(:,i),V(:,i)]=ch5Reference(plan,age+times(i));end
remaining=struct('accepted',true,'t',times,'q',Q,'qd',V);
end
