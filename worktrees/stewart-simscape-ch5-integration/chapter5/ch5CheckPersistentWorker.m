function report=ch5CheckPersistentWorker()
% Compare measured Simscape motion with original preparation, then prove
% task/model reuse and complete state resets in one MATLAB process.
[c,root]=ch5Init();c.episode_seconds=.12;directory=fullfile(root,'chapter5','artifacts','validation','persistent_worker_probe');
if ~isfolder(directory),mkdir(directory);end
saved=load(fullfile(root,'chapter5','artifacts','tasks','development.mat'),'tasks');
tasks=saved.tasks(1:2);tasks{1}.probe_force=[20;-10;15;-15;10;-20];tasks{2}.probe_force=[-10;20;-15;15;-20;10];
report=struct('passed',false,'pid',feature('getpid'),'pairs',{{}},'controller_source_identity',ch5SourceIdentity());
for mode=["ideal-force","nonideal-force"]
    for index=1:2
        task=tasks{index};task.id=sprintf('reuse_probe_%d',index);
        setappdata(0,'ch5ReuseEnabled',false);ch5ReleaseEpisodeData(mode,"hold",task);
        coldFile=fullfile(directory,sprintf('%s_%d_original.mat',strrep(mode,'-','_'),index));
        cold=ch5RunEpisode(task,mode,'hold',0,c,coldFile);
        assert(strcmp(cold.status,'completed'));
        setappdata(0,'ch5ReuseEnabled',true);ch5ReleaseEpisodeData(mode,"hold",task);
        reusedFile=fullfile(directory,sprintf('%s_%d_reused.mat',strrep(mode,'-','_'),index));
        reused=ch5RunEpisode(task,mode,'hold',0,c,reusedFile);
        assert(strcmp(reused.status,'completed'));
        a=load(coldFile,'records','audit');b=load(reusedFile,'records','audit');
        assert(numel(a.records)==numel(b.records)&&numel(a.records)>=7);
        error=0;for j=1:numel(a.records)
            for field={'x','u','q','qd'}
                error=max(error,norm(a.records{j}.(field{1})-b.records{j}.(field{1}),inf));
            end
        end
        auditError=max(abs(a.audit.actual_force-b.audit.actual_force),[],'all');
        poseError=max(abs(a.audit.absolute_pose-b.audit.absolute_pose),[],'all');
        assert(error<1e-9&&auditError<1e-8&&poseError<1e-9,'ch5:ReuseMotionChanged','Repeated preparation changed measured plant motion.');
        if index==2,assert(reused.initialization_detail.model_reused);end
        report.pairs{end+1}=struct('mode',mode,'task',index,'state_command_error',error, ...
            'measured_force_error',auditError,'measured_pose_error',poseError, ...
            'original_initialization_seconds',cold.initialization_seconds, ...
            'reused_initialization_seconds',reused.initialization_seconds,'detail',reused.initialization_detail);
        fprintf('Reuse %s task %d: original %.3fs reused %.3fs error %.3g\n',mode,index,cold.initialization_seconds,reused.initialization_seconds,error);
    end
end
ch5ReleaseEpisodeData();
for name=["ch5Records","ch5Predictions","ch5Plans","ch5Audit","ch5State","ch5Random","ch5DiffusionWarm"]
    assert(~isappdata(0,name));
end
report.passed=true;save(fullfile(directory,'acceptance.mat'),'report','-v7');
fid=fopen(fullfile(directory,'acceptance.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
