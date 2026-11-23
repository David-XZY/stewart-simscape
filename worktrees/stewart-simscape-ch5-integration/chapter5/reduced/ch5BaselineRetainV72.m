function [retained,entry]=ch5BaselineRetainV72(s,q,qd,goal,o,r,time)
% Updating a moving-obstacle plan starts with checking its remaining portion.
% Reuse is only for a task-goal plan with small measured tracking error.
retained=false;entry=struct('time',time,'observed',o,'plan',struct('accepted',false,'attempts',{{}}, ...
    'status','reuse_precheck_not_eligible','seconds',0),'reference_decision','none', ...
    'previous_reference_check',[],'backup_plan',[],'reuse_tracking',[]);
if ~s.plan.accepted||norm(goal-s.goal)>1e-10||~isfield(s.plan,'reference_mode')||~strcmp(s.plan.reference_mode,'task'),return;end
if norm(s.plan.q(:,end)-goal)>1e-8,return;end
age=max(0,time-s.planTime);[qr,vr]=ch5Reference(s.plan,age);
position=norm(q(1:3)-qr(1:3));rotation=acos(max(-1,min(1,(trace(ch5Rotation(qr(4:6))'*ch5Rotation(q(4:6)))-1)/2)));
velocity=norm(qd(1:3)-vr(1:3));omega=norm(rpyRateMapZYX(q(4:6))*qd(4:6)-rpyRateMapZYX(qr(4:6))*vr(4:6));
entry.reuse_tracking=struct('position_m',position,'rotation_rad',rotation,'velocity_mps',velocity,'omega_radps',omega, ...
    'position_limit_m',.001,'rotation_limit_rad',deg2rad(.1),'velocity_limit_mps',.005,'omega_limit_radps',deg2rad(.2));
if position>.001||rotation>deg2rad(.1)||velocity>.005||omega>deg2rad(.2),return;end
times=sort([0,s.plan.t(s.plan.t>age+1e-9)-age,max(r.c.replan_seconds,s.plan.t(end)-age)]);
times=times([true,diff(times)>1e-9]);Q=zeros(6,numel(times));V=Q;
for i=1:numel(times),[Q(:,i),V(:,i)]=ch5Reference(s.plan,age+times(i));end
remaining=struct('accepted',true,'t',times,'q',Q,'qd',V);
timer=tic;checked=ch5SpeedPlanPostcheck(remaining,o,r);seconds=toc(timer);entry.previous_reference_check=checked;
if checked.accepted
    retained=true;entry.reference_decision='retained_task_plan_after_full_updated_obstacle_check';
    entry.plan=struct('accepted',true,'attempts',{{}},'status','checked_remaining_reference_no_new_solve','seconds',seconds);
end
end
