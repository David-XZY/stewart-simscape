function ch5SimscapeSfun(block)
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
    'planTime',-inf,'attemptTime',-inf,'goal',nan(6,1),'q',r.task.q0,'qd',r.task.qd0,'nominal',r.initialForce,'diagnostic',zeros(160,1));
if r.mode=="nonideal-force",state.x=[state.x;repmat(r.initialForce,3,1)];end
if isfield(r.task,'probe_plan')
    assert(strcmp(r.task.split,'development'),'ch5:ProbeScope','Saved-plan probes are restricted to independent development tasks.');
    state.plan=r.task.probe_plan;state.planTime=0;state.attemptTime=0;state.goal=r.task.goals(:,1);
end
setappdata(0,'ch5State',state);setappdata(0,'ch5Records',{});setappdata(0,'ch5Predictions',{});
setappdata(0,'ch5Plans',{});setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',max(1,100000+r.seed+r.task.seed+r.task.index)));
end
function output(block)
r=block.DialogPrm(1).Data;s=getappdata(0,'ch5State');t=block.CurrentTime;
if ~isfinite(s.lastTime)||abs(t-s.lastTime)>1e-10
    [q,qd]=ch5AbsoluteState(block.InputPort(1).Data(:),block.InputPort(2).Data(:),r.framePose);
    if isfinite(s.lastTime),s.x=full(r.plant.fd(s.x,s.force,zeros(6,1)));end
    s.x(1:12)=[q;qd];[goal,o,truth,event]=ch5Observe(r.task,t);
    total=tic;replanSeconds=0;replanned=false;controllerRuntime=rmfield(r,'task');
    if r.method=="baseline"
        moving=any(vecnorm(o(16:18,:))>0);
        replan=~isfinite(s.attemptTime)||norm(goal-s.goal)>1e-10|| ...
            (moving&&t-s.attemptTime>=r.c.replan_seconds-1e-10);
        if replan
            planningTimer=tic;p=ch5Ihsid(q,qd,goal,o,controllerRuntime,6);replanSeconds=toc(planningTimer);s.attemptTime=t;
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
        [u,d,prediction]=ch5DiffusionControl(s.x,s.previous,goal,o,controllerRuntime,t);
        nominal=u;
    elseif r.method=="hold"
        u=r.initialForce;if isfield(r.task,'probe_force'),u=u+r.task.probe_force;end
        nominal=u;d=struct('feasible',true,'usedFallback',false,'exitflag',1,'solveTime',0);
        prediction=[];
    else,error('ch5:Method','Unsupported controller.');end
    seconds=toc(total);
    rec=struct('time',t,'x',s.x,'q',q,'qd',qd,'u',u,'previous_u',s.previous,'nominal',nominal,'goal',goal, ...
        'observed',o,'truth',truth,'event',event,'diagnostic',d,'controller_seconds',seconds, ...
        'planning_seconds',replanSeconds,'replanned',replanned);
    records=getappdata(0,'ch5Records');records{end+1}=rec;setappdata(0,'ch5Records',records);
    if ~isempty(prediction),predictions=getappdata(0,'ch5Predictions');predictions{end+1}=struct('time',t,'prediction',prediction);setappdata(0,'ch5Predictions',predictions);end
    s.force=u;s.previous=u;s.lastTime=t;s.goal=goal;s.q=q;s.qd=qd;s.nominal=nominal;
    s.diagnostic=zeros(160,1);s.diagnostic(1:4)=[d.feasible;d.usedFallback;d.exitflag;seconds];
    setappdata(0,'ch5State',s);
end
block.OutputPort(1).Data=s.force;block.OutputPort(2).Data=s.q;block.OutputPort(3).Data=s.qd;
block.OutputPort(4).Data=s.nominal;block.OutputPort(5).Data=s.diagnostic;
end
