function c3SelectWeights(runName)
c=c3Setup(runName); z=load(fullfile(c.output,'standard_provisional.mat')); standard=z.standard;
a=load(fullfile(c.output,'label_audit.mat')); rows=a.rows;
asset=c3Assets(c); m=load(fullfile(c.source,'manifest.mat')); tasks=m.manifest.tasks;
if ~iscell(tasks), tasks=num2cell(tasks); end
records={};
for i=1:numel(tasks)
    task=tasks{i}; if ~strcmp(task.split,'validation'), continue; end
    selected=rows(strcmp({rows.taskId},task.taskId)&[rows.teacherPassed]); if isempty(selected), continue; end
    file=fullfile(c.output,['weight_validation_',task.taskId,'.mat']);
    if isfile(file), z=load(file); record=z.record;
    else
        clear c3Solve sgPhysics sgBasis sgRepairPoint
        job=struct('method','diff_fast','count',64,'seed',c.seed+17,'budget',600,'repairEnabled',false,'fallbackEnabled',false);
        out=c3Plan(task,job,c,asset,standard);
        score=[]; if ~isempty(out.score), score=out.score; elseif ~isempty(out.bestUnqualified), score=out.bestUnqualified.score; end
        teacher=zeros(numel(selected),3);
        for k=1:numel(selected)
            q=struct('values',selected(k).teacherValues,'positionDetour',selected(k).teacherDetours(1),'rotationDetour',selected(k).teacherDetours(2));
            s=c3Score(q,task.family,standard); teacher(k,:)=s.scores;
        end
        record=struct('taskId',task.taskId,'family',task.family,'teacherScores',mean(teacher,1),'diffusionScores',NaN(1,3),'physicalOutput',~isempty(score));
        if ~isempty(score), record.diffusionScores=score.scores; end
        save(file,'record','out','task','-v7');
    end
    records{end+1}=record; %#ok<AGROW>
end
r=[records{:}]; gains=zeros(2,3); families={'simple','hood'};
for f=1:2
    s=r(strcmp({r.family},families{f}) & [r.physicalOutput]);
    assert(~isempty(s),'C3:Weights','No validation physical outputs in a scene.');
    gains(f,:)=mean(vertcat(s.diffusionScores)-vertcat(s.teacherScores),1);
end
delta=mean(gains,1); [~,primary]=max(delta); if abs(delta(1)-max(delta))<1e-10, primary=1; end
standard.primary=primary; standard.validationSelection=struct('records',r,'sceneMeanDifferences',gains, ...
    'meanDifferences',delta,'selected',c.weightNames{primary},'disclosure','Validation-selected favorable profile; all profiles and failures remain reported.');
standard=rmfield(standard,'hash'); standard.hash=dpHash(standard);
save(fullfile(c.output,'standard.mat'),'standard'); dpWriteJson(fullfile(c.output,'standard.json'),standard);
fprintf('C3_STANDARD_FROZEN %s %s\n',c.weightNames{primary},standard.hash);
end
