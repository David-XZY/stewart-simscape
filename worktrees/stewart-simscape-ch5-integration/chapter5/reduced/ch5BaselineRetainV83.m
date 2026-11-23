function [retained,entry]=ch5BaselineRetainV83(s,q,qd,goal,o,r,time)
% Revalidate the reference and its goal hold over the full prediction horizon.
% This uses only current observed constant-velocity motion, never task truth.
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
hold=max(r.c.replan_seconds,r.c.H*r.c.Ts);
times=sort([0,s.plan.t(s.plan.t>age+1e-9)-age,max(hold,s.plan.t(end)-age)]);
times=times([true,diff(times)>1e-9]);Q=zeros(6,numel(times));V=Q;
for i=1:numel(times),[Q(:,i),V(:,i)]=ch5Reference(s.plan,age+times(i));end
remaining=struct('accepted',true,'t',times,'q',Q,'qd',V);
timer=tic;checked=ch5SpeedPlanPostcheck(remaining,o,r);seconds=toc(timer);
checked.hold_lookahead_seconds=hold;checked.prediction='observed_constant_velocity';entry.previous_reference_check=checked;
if checked.accepted
    retained=true;entry.reference_decision='retained_task_plan_after_horizon_goal_hold_check';
    entry.plan=struct('accepted',true,'attempts',{{}},'status','checked_remaining_reference_and_goal_hold','seconds',seconds);
end
end
