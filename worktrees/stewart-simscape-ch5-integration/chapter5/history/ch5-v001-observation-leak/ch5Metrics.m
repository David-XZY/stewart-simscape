function metrics=ch5Metrics(records,output,r)
assert(~isempty(records),'ch5:MissingSamples','No controller samples were recorded.');
simout=output.get('simout');[time,pose]=signal(simout.x.Xr);[vt,vel]=signal(simout.ch5SpatialVelocity);
vel=interp1(vt,vel,time,'linear');[ft,actual]=signal(simout.y.Taum);actual=interp1(ft,actual,time,'linear');
n=numel(time);position=zeros(n,1);rotation=position;linear=position;angular=position;length=zeros(n,6);speed=length;acc=length;sigma=position;
certificate=inf(n,1);clearance=inf(n,1);collision=false(n,1);distanceVerified=true(n,1);events=zeros(n,1);poses=zeros(n,6);
for k=1:n
    [q,v]=ch5AbsoluteState(pose(k,:)',vel(k,:)',r.framePose);[goal,~,truth,events(k)]=ch5Observe(r.task,time(k));
    poses(k,:)=q';position(k)=norm(q(1:3)-goal(1:3));rotation(k)=acos(max(-1,min(1,(trace(ch5Rotation(goal(4:6))'*ch5Rotation(q(4:6)))-1)/2)));
    linear(k)=norm(v(1:3));angular(k)=norm(vel(k,4:6));
    [~,~,~,~,~,~,Jbar,L,Ld,~,a0,aG]=r.ad.dynamics(q,v);
    length(k,:)=full(L)';speed(k,:)=full(Ld)';acc(k,:)=full(a0+aG*actual(k,:)')';sigma(k)=min(svd(full(Jbar)));
    gap=full(r.collision.value(q,truth));active=find(truth(22,:)>.5);
    if ~isempty(active)
        certificate(k)=min(gap(active));
        for j=active
            [distance,contact,converged]=ch5GjkDistance(q,truth(:,j),r.physical,r.scene);
            clearance(k)=min(clearance(k),distance);collision(k)=collision(k)||contact;distanceVerified(k)=distanceVerified(k)&&converged;
        end
    end
end
ct=cellfun(@(s)s.time,records);commands=cell2mat(cellfun(@(s)s.u,records,'UniformOutput',false));
rate=diff(commands,1,2)/r.c.Ts;
measuredAcceleration=gradientColumns(speed,time);
violations=struct('length',max([r.physical.lmin'-length,length-r.physical.lmax',zeros(n,1)],[],2), ...
    'speed',max([abs(speed)-r.physical.actuator.ldotMax',zeros(n,1)],[],2), ...
    'acceleration',max([abs(acc)-r.physical.actuator.lddotMax',abs(measuredAcceleration)-r.physical.actuator.lddotMax',zeros(n,1)],[],2), ...
    'singularity',max(0,r.strict.sigmaSafe-sigma),'clearance',max(0,r.c.safe_distance-clearance), ...
    'actual_force',max([actual-r.strict.forceMax',r.strict.forceMin'-actual,zeros(n,1)],[],2));
safe=~any(collision)&&all(distanceVerified);
names=fieldnames(violations);vsummary=struct();dt=[diff(time);0];
for j=1:numel(names)
    v=violations.(names{j});vsummary.(names{j})=struct('seconds',sum(dt(v>1e-6)),'maximum',max(v));safe=safe&&max(v)<=1e-6;
end
commandViolation=max([commands-r.strict.forceMax,r.strict.forceMin-commands,zeros(6,1)],[],'all');
rateViolation=max([abs(rate)-r.strict.forceRateLimit,zeros(6,1)],[],'all');safe=safe&&commandViolation<=1e-6&&rateViolation<=1e-6;
arrived=position<=r.c.position_tolerance & rotation<=deg2rad(r.c.rotation_tolerance_degrees)&linear<=r.c.linear_speed_tolerance&angular<=deg2rad(r.c.angular_speed_tolerance_degrees);
tight=position<=r.c.position_tight & rotation<=deg2rad(r.c.rotation_tight_degrees)&linear<=r.c.linear_speed_tolerance&angular<=deg2rad(r.c.angular_speed_tolerance_degrees);
arrival=nan(1,numel(r.task.goal_times));arrivalTight=arrival;
for j=1:numel(arrival)
    valid=events==j;arrival(j)=holdArrival(time,arrived&valid,r.c.arrival_hold_seconds);
    arrivalTight(j)=holdArrival(time,tight&valid,r.c.arrival_hold_seconds);
end
success=isfinite(arrival(end));compute=cellfun(@(s)s.controller_seconds,records);planning=cellfun(@(s)s.planning_seconds,records);replans=planning(2:end);replans=replans(replans>0);
fallback=cellfun(@(s)s.diagnostic.usedFallback,records);failed=cellfun(@(s)~s.diagnostic.feasible,records);
metrics=struct('arrival',success,'safe_success',success&&safe,'tight_arrival',isfinite(arrivalTight(end)), ...
    'all_constraints_valid',safe,'completion_seconds',arrival(end),'rearrival_seconds',arrival-r.task.goal_times, ...
    'position_rms',sqrt(sum(position.^2.*dt)/max(time(end),eps)),'rotation_rms',sqrt(sum(rotation.^2.*dt)/max(time(end),eps)), ...
    'final_position',position(end),'final_rotation',rotation(end),'actual_force_rms',sqrt(sum(sum(actual.^2,2).*dt)/(6*sum(dt))), ...
    'command_rate_rms',sqrt(mean(rate.^2,'all')),'command_force_violation',commandViolation,'command_rate_violation',rateViolation, ...
    'collision',any(collision),'minimum_clearance',min(clearance),'minimum_certificate',min(certificate), ...
    'distance_verified',all(distanceVerified),'violations',vsummary,'fallback_count',sum(fallback),'solver_failure_count',sum(failed), ...
    'controller_seconds',stats(compute),'initial_planning_seconds',planning(1),'replanning_seconds',stats(replans), ...
    'simulated_end_seconds',time(end),'control_samples',numel(ct),'dense_samples',n);
audit=struct('time',time,'absolute_pose',poses,'position_error',position,'rotation_error',rotation,'leg_length',length,'leg_speed',speed, ...
    'leg_acceleration_model_actual_force',acc,'leg_acceleration_measured',measuredAcceleration, ...
    'actual_force',actual,'sigma_actual',sigma,'certificate',certificate,'clearance',clearance,'collision',collision, ...
    'violations',violations,'events',events);
setappdata(0,'ch5Audit',audit);
plans=getappdata(0,'ch5Plans');metrics.planning_attempts=0;metrics.planning_solver_failures=0;metrics.planning_rejections=0;
for j=1:numel(plans)
    metrics.planning_rejections=metrics.planning_rejections+double(~plans{j}.plan.accepted);
    for a=1:numel(plans{j}.plan.attempts)
        attempt=plans{j}.plan.attempts{a};metrics.planning_attempts=metrics.planning_attempts+1;
        metrics.planning_solver_failures=metrics.planning_solver_failures+double(~isfield(attempt,'solver_success')||~attempt.solver_success);
    end
end
predictions=getappdata(0,'ch5Predictions');metrics.predictive_solves=0;metrics.predictive_solver_failures=0;
for j=1:numel(predictions)
    attempts=predictions{j}.prediction.denoise_attempts;
    if isfield(predictions{j}.prediction,'final_attempts'),attempts=[attempts(:);predictions{j}.prediction.final_attempts(:)];end
    for a=1:numel(attempts)
        attempt=attempts{a};metrics.predictive_solves=metrics.predictive_solves+1;
        metrics.predictive_solver_failures=metrics.predictive_solver_failures+double(~isfield(attempt,'solver_success')||~attempt.solver_success);
    end
end
end
function timeFound=holdArrival(t,ok,duration)
timeFound=nan;start=nan;
for i=1:numel(t)
    if ok(i)
        if isnan(start),start=t(i);end
        if t(i)-start>=duration-1e-10,timeFound=t(i);return;end
    else,start=nan;end
end
end
function out=gradientColumns(x,t)
out=zeros(size(x));for j=1:size(x,2),out(:,j)=gradient(x(:,j),t);end
end
function s=stats(x)
if isempty(x),s=struct('median',nan,'p95',nan,'maximum',nan);else,s=struct('median',median(x),'p95',prctile(x,95),'maximum',max(x));end
end
function [t,x]=signal(s)
t=s.Time(:);x=squeeze(s.Data);if size(x,1)~=numel(t),x=x';end
[t,i]=unique(t,'stable');x=x(i,:);
end
