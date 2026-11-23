function report=ch5RepairProbe(index,mode,duration,scenario)
% Independent development prefix; never counted as a development/formal run.
if nargin<3,duration=.32;end
if nargin<4,scenario='original';end
[c,root]=ch5Init();c.episode_seconds=duration;c.checkpoint_steps=1;
s=load(fullfile(root,'chapter5','artifacts','tasks','development.mat'),'tasks');
task=s.tasks{index};identity=ch5SourceIdentity();
if strcmp(scenario,'early_goal')
    assert(size(task.goals,2)>1,'ch5:MissingGoalEvent','This independent event probe requires a goal update.');
    task.goal_times=[0,linspace(.12,.24,size(task.goals,2)-1)];
    task.id=[task.id,'_early_goal_probe'];
end
label=sprintf('repair_%s_%03d_%s_%dms_%s',identity.diffusion_source_sha256(1:8),index,strrep(mode,'-','_'),round(duration*1000),scenario);
file=fullfile(root,'chapter5','artifacts','validation',[label,'.mat']);
result=ch5RunEpisode(task,mode,'diffusion',5101,c,file);
assert(strcmp(result.status,'completed'),'ch5:RepairExecution','Repair probe had an implementation error.');
data=load(file,'records','predictions');records=data.records;
fallbacks=cellfun(@(s)s.diagnostic.usedFallback,records);
statuses={};exceptions=0;physicalAccepted=0;
for j=1:numel(data.predictions)
    p=data.predictions{j}.prediction;
    for k=1:numel(p.denoise_attempts)
        a=p.denoise_attempts{k};statuses{end+1}=a.status;exceptions=exceptions+strcmp(a.status,'exception');
    end
    if p.chosen>0
        candidate=p.candidates{p.chosen};record=records{j};
        assert(p.checks{p.chosen}.accepted);
        assert(norm(candidate.X(:,1)-record.x,inf)<1e-9);
        assert(norm(candidate.U(:,1)-record.u,inf)<1e-9);
        assert(norm(diff([record.previous_u,candidate.U],1,2)-candidate.delta,inf)<1e-9);
        physicalAccepted=physicalAccepted+1;
    end
end
[kinds,~,groups]=unique(string(statuses));counts=accumarray(groups(:),1);
[rr]=deal([records{:}]);goals=[rr.goal];
visible=cell2mat(cellfun(@(s)s.observed(22,:)',records(:)','UniformOutput',false));
goalUpdates=nnz(any(abs(diff(goals,1,2))>1e-10,1));visibilityChanges=nnz(any(diff(visible,1,2)~=0,1));
[user,system]=memory;
report=struct('version',c.version,'control_revision',c.control_revision,'prefix_only',true, ...
    'index',index,'mode',mode,'duration',duration,'control_calls',numel(records), ...
    'fallbacks',sum(fallbacks),'accepted_calls',physicalAccepted,'projection_exceptions',exceptions, ...
    'statuses',{cellstr(kinds)},'status_counts',counts,'memory',user,'system_memory',system, ...
    'all_constraints_valid',result.metrics.all_constraints_valid,'wall_seconds',result.wall_seconds);
report.diffusion_source_sha256=identity.diffusion_source_sha256;report.scenario=scenario;
report.goal_updates_observed=goalUpdates;report.obstacle_visibility_changes=visibilityChanges;
report.passed=exceptions==0&&report.all_constraints_valid&&physicalAccepted>0&&all(cellfun(@(s)all(isfinite(s.u)),records));
if strcmp(scenario,'early_goal'),report.passed=report.passed&&goalUpdates>0;end
save(fullfile(root,'chapter5','artifacts','validation',[label,'_report.mat']),'report','-v7');
fid=fopen(fullfile(root,'chapter5','artifacts','validation',[label,'_report.json']),'w','n','UTF-8');
assert(fid>=0);fwrite(fid,jsonencode(report),'char');fclose(fid);
disp(report);assert(report.passed,'ch5:RepairProbeFailed','Repair prefix did not pass.');
end
