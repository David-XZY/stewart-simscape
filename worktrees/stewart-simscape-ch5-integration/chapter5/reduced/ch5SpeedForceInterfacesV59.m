function report=ch5SpeedForceInterfacesV59()
% Independent directional derivatives and paired reuse checks for the new coordinates.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v57','pilot');
assert(~isfile(fullfile(directory,'interfaces_v59.json')),'ch5:ExistingAttempt','Preserve interface audits.');
s=load(fullfile(directory,'selection.mat'));w=load(fullfile(directory,'seed_5101','weights.mat'));data=load(fullfile(directory,'candidates_5101.mat'));
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('accepted',false,'rows',{{}},'terminal_certified',false);generator=RandStream('mt19937ar','Seed',5901);
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);r.reduced=struct('velocity_weight',1,'factor_derivatives',false,'linearization_collision_reserve',.0002, ...
        'reuse_sensitivities',false,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);k=ch5ReducedKernelsV4(r);
    indices=unique(data.selection_index);indices=indices((s.context(219,indices)>.5)==strcmp(mode,'nonideal-force'));index=indices(1);
    c=double(s.context(:,index));x=c(1:r.plant.n);previous=c(31:36);goal=c(37:42);o=reshape(c(43:218),22,8);D=double(s.delta(:,index));
    [z,U,d]=ch5ForceCodecV59(D,previous,w,'encode');[~,Ud,Dd]=ch5ForceCodecV59(z,previous,w,'decode'); %#ok<ASGLU>
    codec=max(abs(Dd(:)-D));
    terminal=ch5ForceTerminalV57(r.task.q0,r,k);n=r.plant.n;
    [~,A,B]=k.stage(terminal.x,terminal.u);A=full(A);B=full(B);
    Aa=[A,B;zeros(6,n),eye(6)];Ba=[B;eye(6)];v=terminal.scale.*randn(generator,n+6,1);du=randn(generator,6,1);eps=1e-5;
    xp=terminal.x+eps*v(1:n);xm=terminal.x-eps*v(1:n);up=terminal.u+eps*(v(n+1:end)+du);um=terminal.u-eps*(v(n+1:end)+du);
    yp=[full(r.plant.fd(xp,up,zeros(6,1)));up];ym=[full(r.plant.fd(xm,um,zeros(6,1)));um];
    derivative=norm(((yp-ym)/(2*eps)-Aa*v-Ba*du)./terminal.scale,inf);
    direction=randn(generator,6,60);forceDirection=cumsum(direction,2);nodes=full(k.nodes(x,U));[~,aa,bb]=k.stages(nodes(:,1:60),U);aa=full(aa);bb=full(bb);sensitivity=zeros(n,1);
    for j=1:60,sensitivity=aa(:,n*(j-1)+(1:n))*sensitivity+bb(:,6*(j-1)+(1:6))*forceDirection(:,j);end
    ps=[.1*ones(3,1);deg2rad(10)*ones(3,1)];grad=2*(nodes(1:6,end)-goal)./ps.^2;analytic=grad'*sensitivity(1:6);
    np=full(k.nodes(x,U+eps*forceDirection));nm=full(k.nodes(x,U-eps*forceDirection));
    finiteDifference=(sum(((np(1:6,end)-goal)./ps).^2)-sum(((nm(1:6,end)-goal)./ps).^2))/(2*eps);
    taskGradient=abs(analytic-finiteDifference)/max(1,abs(analytic));
    brake=ch5ReducedBrakeSequence(r);inc=full(brake(x,previous));[~,traj]=ch5ForceAuditV58(x,previous,inc,goal,o,r,k,[]);
    if isempty(traj.terminal),[inc,~]=ch5ReducedTerminalV23(x,inc,r,k,traj.X);[~,traj]=ch5ForceAuditV58(x,previous,inc,goal,o,r,k,[]);end
    [~,old]=ch5ReducedStepV50(x,previous,inc,goal,o,zeros(size(inc)),r,k,0,true);new=ch5ForceLinearizeV59(x,previous,inc,goal,o,r,k,0);
    constraintError=max([norm(old.A-new.A,inf),norm(old.b-new.b,inf),norm(old.M-new.M,inf),norm(old.nodes-new.nodes,inf)]);
    ids=find(data.selection_index==index);prior=reshape(double(data.residual(:,ids)),6,60,4)+previous;
    timer=tic;[p58,q58,i58]=ch5ForceStepV58(x,previous,inc,goal,o,prior,r,k,0,traj.artificial_pose);seconds58=toc(timer);
    timer=tic;[p59,q59,i59]=ch5ForceStepV59(x,previous,inc,goal,o,prior,r,k,0,traj.artificial_pose);seconds59=toc(timer);
    proposalError=max([max(abs(p58-p59),[],'all'),max(abs(q58-q59),[],'all')]);
    valid=codec<1e-12&&derivative<1e-5&&taskGradient<1e-5&&constraintError<1e-9&&proposalError<1e-6;
    report.rows{end+1}=struct('mode',char(mode),'codec_error',codec,'augmented_derivative_error',derivative,'task_gradient_relative_error',taskGradient, ...
        'shared_constraint_error',constraintError,'shared_proposal_error',proposalError,'v58_seconds',seconds58,'v59_seconds',seconds59,'accepted',valid);
end
report.accepted=all(cellfun(@(v)v.accepted,report.rows));
fid=fopen(fullfile(directory,'interfaces_v59.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(report.accepted,'ch5:ForceInterface','New coordinates or shared derivative arithmetic failed.');
end
