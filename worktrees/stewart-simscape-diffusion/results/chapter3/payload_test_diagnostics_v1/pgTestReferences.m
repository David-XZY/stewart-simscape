function summary=pgTestReferences(manifestPath,runName,maxNew,wallHours)
% Resumable train/validation reference production. Never reads test references.
if nargin<3, maxNew=2880; end
if nargin<4, wallHours=62; end
c=pgConfig(runName); manifest=jsondecode(fileread(manifestPath)); specs=manifest.tasks;
assert(all(ismember({specs.split},{'test'})),'PG:Leakage','Only frozen test tasks allowed.');
identity=struct('manifestHash',dpHash(fileread(manifestPath)),'codeHash',c.codeHash,'wallHours',wallHours,'executorHash',dpHash(fileread([mfilename('fullpath'),'.m'])));
idfile=fullfile(c.output,'identity.json');
if isfile(idfile), assert(isequal(jsondecode(fileread(idfile)),identity),'PG:Identity','Changed source or manifest requires new run.');
else, dpWriteJson(idfile,identity); copyfile(manifestPath,fullfile(c.output,'frozen_tasks.json')); end
completed=false(numel(specs),8); modes=cell(numel(specs),1); count=0; qualified=0; spent=0;
files=dir(fullfile(c.output,'attempt_*.mat'));
for k=1:numel(files)
 s=load(fullfile(files(k).folder,files(k).name)); i=find(strcmp({specs.taskId},s.row.taskId));
 assert(numel(i)==1,'PG:Identity','Unknown saved task.'); completed(i,s.row.startId)=true; count=max(count,s.row.attempt); spent=spent+s.row.actualAttemptSeconds;
 if s.row.passed
  qualified=qualified+1; f=pgModeFeatures(s.sol.bundle,s.task);
  if isempty(modes{i})||min(sqrt(mean((modes{i}-f).^2,1)))>.15, modes{i}=[modes{i},f]; end
 end
end
started=tic; added=0; stop=false;
for seedId=1:4
 for i=1:numel(specs)
  if added>=maxNew||spent+toc(started)+125>wallHours*3600, stop=true; break; end
  if completed(i,seedId)||(strcmp(specs(i).split,'validation')&&seedId>4)||(seedId>4&&size(modes{i},2)>=2), continue; end
  timer=tic; task=pgMakeTask(specs(i),c); sol=[]; count=count+1; added=added+1;
  row=struct('attempt',count,'taskId',task.taskId,'baseSceneId',task.baseSceneId,'split',task.split,'shape',task.payload.shape,'mass',task.payload.mass,'T',task.T,'startId',seedId,'endpointPassed',task.endpointPassed,'passed',false,'reason',task.endpointReason,'codeHash',c.codeHash);
  if task.endpointPassed
   cc=c; cc.clockHandle=timer; cc.wallBudget=120; cc.maxCpuSeconds=120;
   try
    sol=pgSolve(task,bdSeed(task,seedId,c),cc); row.passed=sol.passed; row.reason=sol.solverStatus;
    row.solverSuccess=sol.solverSuccess; row.maxEquality=sol.maxEquality; row.maxInequality=sol.maxInequality; row.maxBounds=sol.maxBounds;
    row.physicalPassed=sol.report.passed; row.physicalReason=sol.report.reason; row.buildSeconds=sol.buildSeconds; row.solveSeconds=sol.solveSeconds;
   catch e, row.reason=[e.identifier,':',e.message]; row.exceptionStack=getReport(e,'extended','hyperlinks','off'); end
  end
  row.actualAttemptSeconds=toc(timer); row.passed=row.passed&&row.actualAttemptSeconds<=120; row.newMode=false;
  if row.passed
   qualified=qualified+1; f=pgModeFeatures(sol.bundle,task); row.newMode=isempty(modes{i})||min(sqrt(mean((modes{i}-f).^2,1)))>.15;
   if row.newMode&&size(modes{i},2)<4, modes{i}=[modes{i},f]; end
  end
  stem=sprintf('attempt_%05d',count); row.trajectoryRef=[stem,'.mat'];
  temp=fullfile(c.output,[stem,'.pending.mat']); save(temp,'task','sol','row','-v7'); movefile(temp,fullfile(c.output,[stem,'.mat']));
  dpWriteJson(fullfile(c.output,[stem,'.json']),row); completed(i,seedId)=true;
  dpWriteJson(fullfile(c.output,'progress.json'),struct('attempts',count,'strictSuccesses',qualified,'resourceSeconds',spent+toc(started),'last',row));
 end
 if stop, break; end
end
summary=struct('attempts',count,'strictSuccesses',qualified,'newAttempts',added,'resourceSeconds',spent+toc(started),'modeCounts',cellfun(@(x)size(x,2),modes),'codeHash',c.codeHash,'status','checkpoint','formalTestReferencesStarted',true);
if ~stop, summary.status='completed'; end
dpWriteJson(fullfile(c.output,'reference_summary.json'),summary);
end
