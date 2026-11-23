function report=ch5SpeedCollisionAuditV22()
clear ch5ReducedCollisionAuditV22
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'collision_audit_v22');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve every audit.');mkdir(directory);
identity=ch5ReducedIdentity();contract=jsondecode(fileread(fullfile(base,'prior_cast_audit_v21','summary.json')));
assert(contract.accepted,'ch5:Unverified','Prior/conditioning contracts must pass.');
for field={'ch5ReducedJacobianV17_m','ch5ReducedStepV20_m','ch5ReducedSolveV19_m','ch5ReducedPriorV21_m'}
    name=field{1};assert(strcmp(identity.(name),contract.identity.(name)),'ch5:StaleAudit','Changed accepted numerical core.');
end
old=fileread(fullfile(root,'chapter5','reduced','ch5ReducedControlV21.m'));fresh=fileread(fullfile(root,'chapter5','reduced','ch5ReducedControlV22.m'));
expected=strrep(strrep(strrep(old,'ch5ReducedControlV21','ch5ReducedControlV22'),'ch5ReducedAuditV12','ch5ReducedAuditV22'),'ch5ReducedHoldAudit','ch5ReducedHoldAuditV22');
assert(strcmp(fresh,expected),'ch5:ChangedController','Collision-only change must be explicit.');
report=struct('accepted',true,'identity',identity,'pid',feature('getpid'),'rows',{{}},'hold_rows',{{}},'gate_rows',{{}});
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepare(4,mode);r.reduced=struct('factor_derivatives',false,'velocity_weight',1);k=ch5ReducedKernelsV4(r);
    source=load(fullfile(base,'closed_conditioned_v21',sprintf('004_%s.mat',strrep(mode,'-','_'))),'records','predictions');
    sample=source.records{12};[~,trace]=r.plant.fd(sample.x,sample.u,zeros(6,1));hold=r;hold.c.H=1;
    empty=sample.observed;empty(22,:)=0;timer=tic;ch5ReducedCollisionAuditV22(full(trace),empty,hold);coldSeconds=toc(timer);
    for mask={[],1,[1,3],[2,4,6],1:r.c.max_obstacles}
        observed=sample.observed;observed(22,:)=0;observed(22,mask{1})=1;
        for oi=1:r.c.max_obstacles
            observed(1:3,oi)=[.2;-.1;1.2]+[.02;-.01;.01]*oi;observed(4:6,oi)=[.05;.08;.03];
            observed(7:15,oi)=reshape(ch5Rotation([.1;.2;.3]*oi),9,1);observed(16:18,oi)=[.03;-.02;.01];
        end
        timer=tic;new=ch5ReducedCollisionAuditV22(full(trace),observed,hold);seconds=toc(timer);oldGap=scalarGap(full(trace),observed,hold);
        error=abs(new-oldGap);if isinf(new)&&isinf(oldGap),error=0;end
        row=struct('mode',char(mode),'active',mask{1},'cold_prepare_seconds',coldSeconds,'seconds',seconds,'error',error,'accepted',error<1e-10);
        report.rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
    end
    % The exact old one-second sample remains in V21; replay the same gate.
    oldCheck=ch5ReducedHoldAudit(sample.x,sample.previous_u,sample.u,sample.observed,r);timer=tic;
    newCheck=ch5ReducedHoldAuditV22(sample.x,sample.previous_u,sample.u,sample.observed,r);seconds=toc(timer);
    row=struct('mode',char(mode),'time',sample.time,'seconds',seconds,'original_seconds',sample.controller_seconds, ...
        'violation_error',abs(newCheck.inequality_violation-oldCheck.inequality_violation),'accepted',newCheck.accepted==oldCheck.accepted&&abs(newCheck.inequality_violation-oldCheck.inequality_violation)<1e-10);
    report.hold_rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
    sample=source.records{276};U=source.predictions{276}.prediction.candidates{1}.U;prior=16*ones(6,r.c.H);
    [u,oldCheck,t,oldCost]=ch5ReducedAuditV12(sample.x,sample.previous_u,U,sample.goal,sample.observed,r,k,prior);
    [newU,newCheck,newT,newCost]=ch5ReducedAuditV22(sample.x,sample.previous_u,U,sample.goal,sample.observed,r,k,prior);
    [~,~,fullTrace]=k.preview(sample.x,u);newGap=ch5ReducedCollisionAuditV22(full(fullTrace),sample.observed,r);ref=scalarGap(full(fullTrace),sample.observed,r);assert(abs(newGap-ref)<1e-10,'ch5:InnerTrace','Full inner mesh changed.');
    errors=[norm(u-newU,inf),norm(t.X-newT.X,inf),abs(oldCost-newCost),abs(oldCheck.inequality_violation-newCheck.inequality_violation)];
    row=struct('mode',char(mode),'errors',errors,'accepted',oldCheck.accepted==newCheck.accepted&&max(errors)<1e-10);
    report.gate_rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
end
save(fullfile(directory,'audit.mat'),'report','-v7');fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(report.accepted,'ch5:CollisionMismatch','Preserve all mismatches.');
end
function gap=scalarGap(trace,o,r)
times=reshape(r.plant.traceTime(:)+(0:r.c.H-1)*r.c.Ts,1,[]);gap=inf;
for oi=find(o(22,:)>.5)
    for j=1:numel(times)
        observed=o(:,oi);observed(1:3)=observed(1:3)+observed(16:18)*times(j);
        gap=min(gap,full(r.collision.single(trace(1:6,j),observed)));
    end
end
end
