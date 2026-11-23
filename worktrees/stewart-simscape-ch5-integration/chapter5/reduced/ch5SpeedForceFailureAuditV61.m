function report=ch5SpeedForceFailureAuditV61()
% Verify goal clearance and isolate terminal/kinematic conflicts using saved observed data.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v57','pilot');
data=load(fullfile(directory,'closed_loop_v60','004_nonideal_force.mat'),'records','predictions');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
r=ch5ReducedPrepareV48(4,'nonideal-force');r.reduced=struct('velocity_weight',1,'factor_derivatives',false);k=ch5ReducedKernelsV4(r);
report=struct('rows',{{}},'terminal_timings',[],'required_gap',r.c.safe_distance+r.c.sample_hold_margin);
for index=[1,51,101,151,251,401,601,681]
    rec=data.records{index};p=data.predictions{index}.prediction;tr=p.candidates{1};o=rec.observed;active=find(o(22,:)>.5);
    gaps=inf(3,2);q=[rec.goal,rec.q,tr.artificial_pose];
    for j=1:3
        for offset=[0,1.2]
            oo=o;oo(1:3,:)=oo(1:3,:)+oo(16:18,:)*offset;
            gg=inf;for a=active,gg=min(gg,full(k.collision(q(:,j),oo(:,a))));end
            gaps(j,1+round(offset/1.2))=gg;
        end
    end
    tg=ch5ForceTerminalV57(rec.goal,r,k);timer=tic;ta=ch5ForceTerminalV57(tr.artificial_pose,r,k);seconds=toc(timer);
    report.rows{end+1}=struct('time',rec.time,'goal_actual_terminal_gaps',gaps,'goal_terminal_valid',tg.numerically_valid, ...
        'goal_rho',tg.rho,'artificial_rho',ta.rho,'terminal_build_seconds',seconds, ...
        'terminal_state_error',tr.X(:,end)-ta.x,'goal_velocity_terminal',tr.X(7:12,end));
end
fid=fopen(fullfile(directory,'failure_audit_v61.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
