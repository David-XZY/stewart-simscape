function report=ch5SpeedTeacherIntervalAuditV62()
clear ch5TeacherIntervalProjectV62
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62');
assert(~isfile(fullfile(directory,'teacher_interval_audit.mat')),'ch5:ExistingAttempt','Preserve earlier attempt.');
s=load(fullfile(directory,'onset_snapshots.mat'),'snapshots');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'rows',{{}},'completed',false,'formal',false);
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);
    for i=1:numel(s.snapshots)
        z=s.snapshots{i};if ~strcmp(z.mode,mode),continue;end
        z.x=double(z.x(:));z.previous=double(z.previous(:));z.u=double(z.u(:));z.nominal=double(z.nominal(:));
        z.cbf.A=double(z.cbf.A);z.cbf.b=double(z.cbf.b(:));z.clf.A=double(z.clf.A);z.clf.b=double(z.clf.b);
        % scipy cannot round-trip MATLAB string MCOS; labels do not alter A,b.
        z.cbf.names="teacher_cbf_"+string((1:size(z.cbf.A,1))');
        [before,~]=ch5ReducedHoldAuditV22(z.x,z.previous,z.u,z.observed,r);r.strict.time=z.time;
        timer=tic;[u,d]=ch5TeacherIntervalProjectV62(z.x,z.previous,z.nominal,z.observed,r,z.clf,z.cbf,z.u);seconds=toc(timer);
        report.rows{end+1}=struct('source',z.source,'time',z.time,'first_violation_time',z.first_violation_time,'mode',char(mode), ...
            'before',before,'after',d.interval,'feasible',d.feasible,'used_fallback',d.usedFallback,'seconds',seconds, ...
            'maximum_force_change_N',norm(u-z.u,inf),'attempts',{d.attempts});
        if mod(numel(report.rows),6)==0
            fid=fopen(fullfile(directory,'teacher_interval_audit.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
        end
    end
end
report.completed=true;save(fullfile(directory,'teacher_interval_audit.mat'),'report','-v7');
fid=fopen(fullfile(directory,'teacher_interval_audit.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('V62 teacher sampled interval repair %d/%d\n',nnz(cellfun(@(v)v.feasible,report.rows)),numel(report.rows));
end
