function ch5ReducedV36Sfun(block)
block.NumDialogPrms=1;block.NumInputPorts=6;block.NumOutputPorts=5;
for j=1:6,block.InputPort(j).Dimensions=6;block.InputPort(j).DirectFeedthrough=true;end
for j=1:5,block.OutputPort(j).Dimensions=6;end
block.OutputPort(5).Dimensions=160;
block.SampleTimes=[block.DialogPrm(1).Data.c.Ts,0];block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('InitializeConditions',@initialize);block.RegBlockMethod('Outputs',@output);
end
function initialize(block)
r=block.DialogPrm(1).Data;
state=struct('previous',r.initialForce,'lastTime',-inf,'force',r.initialForce,'x',[r.task.q0;r.task.qd0], ...
    'lqi',zeros(6,1),'aw',zeros(6,1),'plan',struct('accepted',false), ...
    'planTime',-inf,'attemptTime',-inf,'arrivalTime',6,'goal',nan(6,1),'q',r.task.q0,'qd',r.task.qd0,'nominal',r.initialForce,'diagnostic',zeros(160,1));
state.observed=zeros(22,r.c.max_obstacles);state.observed(7:15,:)=repmat(reshape(eye(3),9,1),1,r.c.max_obstacles);
state.savedRecords=0;state.savedPredictions=0;state.savedPlans=0;
if r.mode=="nonideal-force",state.x=[state.x;repmat(r.initialForce,3,1)];end
if isfield(r.task,'probe_plan')
    assert(strcmp(r.task.split,'development'),'ch5:ProbeScope','Saved-plan probes are restricted to independent development tasks.');
    state.plan=r.task.probe_plan;state.planTime=0;state.attemptTime=0;state.goal=r.task.goals(:,1);
end
if isappdata(0,'ch5ReducedExecution'),rmappdata(0,'ch5ReducedExecution');end
if isappdata(0,'ch5ReducedSensitivity'),rmappdata(0,'ch5ReducedSensitivity');end
setappdata(0,'ch5State',state);setappdata(0,'ch5Records',{});setappdata(0,'ch5Predictions',{});
setappdata(0,'ch5Plans',{});setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',max(1,100000+r.seed+r.task.seed+r.task.index)));
end
function output(block)
r=block.DialogPrm(1).Data;s=getappdata(0,'ch5State');t=block.CurrentTime;
if ~isfinite(s.lastTime)||abs(t-s.lastTime)>1e-10
    total=tic;
    [q,qd]=ch5AbsoluteState(block.InputPort(1).Data(:),block.InputPort(2).Data(:),r.framePose);
    if isfinite(s.lastTime),s.x=full(r.plant.fd(s.x,s.force,zeros(6,1)));end
    s.x(1:12)=[q;qd];[goal,o,truth,event]=ch5Observe(r.task,t);
    replanSeconds=0;replanned=false;controllerRuntime=rmfield(r,'task');
    if r.method=="baseline"
        moving=any(vecnorm(o(16:18,o(22,:)>.5))>0);
        appeared=any(o(22,:)~=s.observed(22,:));
        shapeChanged=any(o(4:15,:)~=s.observed(4:15,:),'all');
        stationary=find(o(22,:)>.5&vecnorm(o(16:18,:))<1e-12);
        stationaryChanged=any(abs(o(1:3,stationary)-s.observed(1:3,stationary))>1e-10,'all');
        replan=~isfinite(s.attemptTime)||norm(goal-s.goal)>1e-10|| ...
            appeared||shapeChanged||stationaryChanged||(moving&&t-s.attemptTime>=r.c.replan_seconds-1e-10);
        if replan
            % Preserve the intended arrival time across periodic replans.
            % A new goal, or an expired planning budget, receives up to six
            % seconds, capped by the registered episode deadline and hold.
            deadline=r.c.episode_seconds-r.c.arrival_hold_seconds;
            if norm(goal-s.goal)>1e-10||s.arrivalTime-t<.5
                s.arrivalTime=min(t+6,deadline);
            end
            duration=s.arrivalTime-t;planningTimer=tic;
            if duration>=.05
                p=ch5SpeedIhsid(q,qd,goal,o,controllerRuntime,duration,t);
            else
                p=struct('accepted',false,'attempts',{{}},'status','arrival_budget_exhausted','seconds',0);
            end
            replanSeconds=toc(planningTimer);s.attemptTime=t;
            plans=getappdata(0,'ch5Plans');plans{end+1}=struct('time',t,'observed',o,'plan',p);setappdata(0,'ch5Plans',plans);
            if p.accepted,s.plan=p;s.planTime=t;replanned=true;
            elseif ~s.plan.accepted
                s.plan=struct('accepted',false,'t',[0,6],'q',[q,q],'qd',zeros(6,2));s.planTime=t;
            end
        end
        [qr,vr,ar]=ch5Reference(s.plan,max(0,t-s.planTime));
        [d,G]=r.ad.dynamics(qr,vr);ff=full(G)\(ar-full(d));
        poseError=[qr(1:3)-q(1:3);atan2(sin(qr(4:6)-q(4:6)),cos(qr(4:6)-q(4:6)))];
        stateError=[-poseError;qd-vr];
        raw=r.lqi.mapping*(-r.lqi.Kx*stateError-r.lqi.Ki*s.lqi);feedback=min(max(raw,-120),120);
        generalizedDifference=r.lqi.mapping\(feedback-raw);
        s.lqi=r.lqi.leak*s.lqi-r.c.Ts*poseError+r.c.Ts*r.lqi.antiwindup*generalizedDifference;
        s.aw=feedback-raw;nominal=ff+feedback;
        [u,d]=ch5SafetyQp(q,qd,qr,vr,ar,nominal,s.previous,o,controllerRuntime,t);
        prediction=[];
    elseif r.method=="diffusion"
        [u,d,prediction]=ch5ReducedControlV36(s.x,s.previous,goal,o,controllerRuntime,t);
        nominal=u;
    elseif r.method=="hold"
        u=r.initialForce;if isfield(r.task,'probe_force'),u=u+r.task.probe_force;end
        nominal=u;d=struct('feasible',true,'usedFallback',false,'exitflag',1,'solveTime',0);
        prediction=[];
    else,error('ch5:Method','Unsupported controller.');end
    if isfield(r,'speed')&&isfield(r.speed,'stop_on_unsafe')&&r.speed.stop_on_unsafe
        force=u;if r.mode=="nonideal-force",force=s.x(19:24);end
        values=full(r.plant.mechanicalValues(s.x,force));
        lower=[r.physical.lmin;-r.physical.actuator.ldotMax;-r.physical.actuator.lddotMax;r.strict.sigmaSafe^2;r.strict.forceMin];
        upper=[r.physical.lmax;r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;inf;r.strict.forceMax];
        if ~all(isfinite(values))||max([lower-values;values-upper;0])>1e-5
            error('ch5:SpeedUnsafe','Diagnostic simulation stopped at %.6fs after a physical constraint violation.',t);
        end
    end
    seconds=toc(total);
    rec=struct('time',t,'x',s.x,'q',q,'qd',qd,'u',u,'previous_u',s.previous,'nominal',nominal,'goal',goal, ...
        'observed',o,'truth',truth,'event',event,'diagnostic',d,'controller_seconds',seconds, ...
        'planning_seconds',replanSeconds,'replanned',replanned);
    records=getappdata(0,'ch5Records');records{end+1}=rec;setappdata(0,'ch5Records',records);
    if ~isempty(prediction),predictions=getappdata(0,'ch5Predictions');predictions{end+1}=struct('time',t,'prediction',prediction);setappdata(0,'ch5Predictions',predictions);end
    s.force=u;s.previous=u;s.lastTime=t;s.goal=goal;s.q=q;s.qd=qd;s.nominal=nominal;s.observed=o;
    if isfield(r,'journalDirectory')&&mod(numel(records),r.c.checkpoint_steps)==0
        newRecords=records(s.savedRecords+1:end);predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        newPredictions=predictions(s.savedPredictions+1:end);newPlans=plans(s.savedPlans+1:end);
        file=fullfile(r.journalDirectory,sprintf('chunk_%05d.mat',numel(records)/r.c.checkpoint_steps));
        pending=[file,'.pending'];save(pending,'newRecords','newPredictions','newPlans','-v7');movefile(pending,file);
        s.savedRecords=numel(records);s.savedPredictions=numel(predictions);s.savedPlans=numel(plans);
    end
    s.diagnostic=zeros(160,1);s.diagnostic(1:4)=[d.feasible;d.usedFallback;d.exitflag;seconds];
    setappdata(0,'ch5State',s);
end
block.OutputPort(1).Data=s.force;block.OutputPort(2).Data=s.q;block.OutputPort(3).Data=s.qd;
block.OutputPort(4).Data=s.nominal;block.OutputPort(5).Data=s.diagnostic;
end
