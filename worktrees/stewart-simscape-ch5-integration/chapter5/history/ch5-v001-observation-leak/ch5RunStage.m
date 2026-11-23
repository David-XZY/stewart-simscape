function summary=ch5RunStage(stage,selection)
% A selection partitions execution only; task generation and protocol stay fixed.
if nargin<2,selection=struct();end
[c,root]=ch5Init();base=fullfile(root,'chapter5','artifacts');
switch stage
    case 'validate',summary=ch5Validate();return;
    case 'teacher',splits=["train","validation"];directory=fullfile(base,'teacher');seeds=0;methods="baseline";
    case 'development',splits="development";directory=fullfile(base,'development');seeds=[0;c.training_seeds(:)];methods=["baseline";repmat("diffusion",5,1)];
    case 'formal'
        assert(isfile(fullfile(base,'freeze.json')),'ch5:NotFrozen','Freeze all development artifacts before test generation.');
        ch5VerifyFreeze();splits="test";directory=fullfile(base,'formal');seeds=[0;c.training_seeds(:)];methods=["baseline";repmat("diffusion",5,1)];
    otherwise,error('ch5:Stage','Unknown stage.');
end
gate=load(fullfile(base,'validation','interface.mat'),'report');assert(gate.report.passed);
for profile=["ideal_force","nonideal_force"]
    old=load(fullfile(base,'regression','strict_qp_'+profile+'.mat'),'report');assert(old.report.passed);
end
[physical,scene]=ch5Physics(c);collision=ch5CollisionModel(physical,scene,c);allTasks={};
for split=splits
    taskFile=fullfile(base,'tasks',split+'.mat');
    if isfile(taskFile),s=load(taskFile,'tasks','version');assert(strcmp(s.version,c.version));tasks=s.tasks;
    else,tasks=ch5GenerateTasks(char(split),c,physical,scene,collision);version=c.version;save(taskFile,'tasks','version');end
    allTasks=[allTasks;tasks];
end
indices=1:numel(allTasks);modes=["ideal-force","nonideal-force"];
if isfield(selection,'indices'),indices=selection.indices;end
if isfield(selection,'modes'),modes=string(selection.modes);end
assert(all(ismember(indices,1:numel(allTasks)))&&numel(unique(indices))==numel(indices));
assert(all(ismember(modes,["ideal-force","nonideal-force"])));
worker="main";if isfield(selection,'worker'),worker=string(selection.worker);end
assert(~isempty(regexp(char(worker),'^[A-Za-z0-9_-]+$','once')));
summary=struct('stage',stage,'worker',worker,'version',c.version,'expected_runs',numel(indices)*numel(modes)*numel(seeds), ...
    'completed',0,'simulation_failed',0,'setup_failed',0);
statusFile=fullfile(base,sprintf('%s_%s_status.json',stage,worker));
for mode=modes
    for methodIndex=1:numel(seeds)
        seed=seeds(methodIndex);method=methods(min(methodIndex,numel(methods)));
        for index=indices
            task=allTasks{index};name=sprintf('%s_%s_%s_%d',task.id,strrep(mode,'-','_'),method,seed);
            result=ch5RunEpisode(task,mode,method,seed,c,fullfile(directory,[name,'.mat']));
            summary.(result.status)=summary.(result.status)+1;
            fid=fopen([statusFile,'.pending'],'w','n','UTF-8');fprintf(fid,'%s',jsonencode(summary));fclose(fid);movefile([statusFile,'.pending'],statusFile,'f');
            if strcmp(result.status,'setup_failed'),error('ch5:SetupFailed','Setup failed for %s; campaign paused before dependent work.',name);end
        end
    end
end
whole=numel(indices)==numel(allTasks)&&numel(modes)==2;
if strcmp(stage,'teacher')&&whole,ch5ExportDataset();end
if strcmp(stage,'formal')&&whole,assert(summary.completed+summary.simulation_failed==1440,'ch5:CountMismatch','The complete formal campaign must have 1440 explicit results.');end
close_system('stewart_platform_model',0);
end
