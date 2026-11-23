function report=ch5SpeedForceKernelAuditV62()
% Fixed measured snapshots, including nonsteady actuator and branch cases.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62');
assert(~isfile(fullfile(directory,'kernel_audit.mat')),'ch5:ExistingAttempt','Preserve earlier kernel audit.');
s=load(fullfile(directory,'teacher_snapshots.mat'),'snapshots');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'rows',{{}},'timings',{{}},'accepted',false,'formal',false);
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);r.reduced=struct('reuse_sensitivities',false,'factor_derivatives',false);
    k=ch5ReducedKernelsV4(r);
    if mode=="nonideal-force",factored=ch5ReducedFactorStage(r);else,factored=k.stage;end
    for i=1:numel(s.snapshots)
        z=s.snapshots{i};if ~strcmp(z.mode,mode),continue;end
        variants={z.u,z.previous};
        if mode=="nonideal-force"
            variants=[variants,{z.x(25:30),z.x(13:18)+r.plant.profile.slewRate*r.c.Ts,z.x(13:18)-r.plant.profile.slewRate*r.c.Ts}];
        end
        for v=1:numel(variants)
            u=variants{v};[y,A,B]=factored(z.x,u);[yr,Ar,Br]=k.stage(z.x,u);
            errors=[norm(full(y-yr),inf),norm(full(A-Ar),inf),norm(full(B-Br),inf)];
            physicalError=norm(full(y)-full(r.plant.fd(z.x,u,zeros(6,1))),inf);
            report.rows{end+1}=struct('source',z.source,'time',z.time,'mode',char(mode),'variant',v,'errors',errors, ...
                'physical_primal_error',physicalError,'accepted',max(errors)<1e-8&&physicalError<1e-8);
        end
    end
    z=s.snapshots{find(cellfun(@(a)strcmp(a.mode,mode),s.snapshots),1)};
    brake=ch5ReducedBrakeSequence(r);U=full(brake(z.x,z.previous));nodes=full(k.nodes(z.x,U));
    mapped=factored.map(r.c.H);old=zeros(1,7);fast=old;
    for j=1:8
        timer=tic;[~,~,~]=k.stages(nodes(:,1:r.c.H),U);a=toc(timer);
        timer=tic;[~,~,~]=mapped(nodes(:,1:r.c.H),U);b=toc(timer);
        if j>1,old(j-1)=a;fast(j-1)=b;end
    end
    report.timings{end+1}=struct('mode',char(mode),'original',old,'factorized',fast);
end
report.accepted=all(cellfun(@(v)v.accepted,report.rows));
save(fullfile(directory,'kernel_audit.mat'),'report','-v7');
fid=fopen(fullfile(directory,'kernel_audit.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('V62 kernel audit %d samples, accepted=%d\n',numel(report.rows),report.accepted);
assert(report.accepted,'ch5:KernelMismatch','Factorized primal and derivative agreement failed.');
end
