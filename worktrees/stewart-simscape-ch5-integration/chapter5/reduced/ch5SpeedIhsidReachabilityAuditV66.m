function report=ch5SpeedIhsidReachabilityAuditV66()
% Fixed post-clear teacher planning states; no full experiment rerun required.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62');
assert(~isfile(fullfile(directory,'ihsid_reachability_v66.json')),'ch5:ExistingAttempt','Preserve reachability audit.');
report=struct('pid',feature('getpid'),'completed',false,'formal',false,'rows',{{}});
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
for mode=["ideal-force","nonideal-force"]
    d=load(fullfile(directory,'teacher_v65',sprintf('train_004_%s.mat',strrep(mode,'-','_'))),'records');r=ch5ReducedPrepareV48(4,mode);
    times=cellfun(@(z)z.time,d.records);
    for targetTime=12.6:.2:14.4
        [~,index]=min(abs(times-targetTime));rec=d.records{index};T=14.5-rec.time;
        timer=tic;bound=ch5IhsidReachabilityV66(rec.q,rec.qd,rec.goal,T,r);seconds=toc(timer);
        report.rows{end+1}=struct('mode',char(mode),'time',rec.time,'bound',bound,'seconds',seconds);
    end
end
report.completed=true;report.rejected=nnz(cellfun(@(z)z.bound.infeasible,report.rows));
save(fullfile(directory,'ihsid_reachability_v66.mat'),'report','-v7');fid=fopen(fullfile(directory,'ihsid_reachability_v66.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
