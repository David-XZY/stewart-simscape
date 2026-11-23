function bdBenchmark(jobsFile,runName)
c=bdConfig(runName); protocol=jsondecode(fileread(jobsFile)); jobs=protocol.jobs;

c.maxCpuSeconds=10;
if isfield(protocol,'controlCount'), c.controlCount=protocol.controlCount; end
if isfield(protocol,'qualityCaps')
    c.positionCap=protocol.qualityCaps(1); c.rotationCap=protocol.qualityCaps(2); c.qualityFrozen=true;
end
if isfield(protocol,'stageFractions'), c.stageFractions=protocol.stageFractions(:).'; end
startup=tic; assets=struct();
if isfield(protocol,'libraryFile') && ~isempty(protocol.libraryFile), assets=bdLoadLibrary(protocol.libraryFile,'train',c); end
% Warmup is task independent; parameter graph and model loads are separately logged.
warm=bdBuildProblem(c); models=struct();
if isfield(protocol,'models')
    entries=protocol.models; if iscell(entries), entries=[entries{:}]; end
    for k=1:numel(entries), models.(entries(k).key)=jsondecode(fileread(entries(k).file)); end
end
warm.wallSeconds=toc(startup); dpWriteJson(fullfile(c.output,'resident_startup.json'),warm);
baseConfig=c;
for k=1:numel(jobs)
    c=baseConfig;
    if iscell(jobs), job=jobs{k}; else, job=jobs(k); end
    if isfield(job,'stageFractions') && ~isempty(job.stageFractions), c.stageFractions=job.stageFractions(:).'; end
    folder=fullfile(c.output,'calls',job.runId); if ~isfolder(folder), mkdir(folder); end
    file=fullfile(folder,'result.json'); if isfile(file), old=jsondecode(fileread(file)); assert(isfield(old,'codeHash')&&strcmp(old.codeHash,c.codeHash),'BD:VersionChanged','Use a new benchmark version after source changes.'); continue; end
    % Task assembly is measured, except loading immutable model/index assets.
    outer=tic;
    dpWriteJson(fullfile(c.output,'active_call.json'),struct('state','running','codeHash',c.codeHash,'startedUnix',posixtime(datetime('now','TimeZone','UTC')),'job',job));
    task=bdMakeTask(job,job.parameters,c); local=assets;
    if isfield(job,'modelKey') && ~isempty(job.modelKey), local.model=models.(job.modelKey); end
    for option={'distanceVariant','localNeighbors','ridge'}, if isfield(job,option{1}) && ~isempty(job.(option{1})), local.(option{1})=job.(option{1}); end; end
    c.returnPolicy=job.returnPolicy;
    preparation=toc(outer); remaining=max(0,job.B-preparation);
    result=bdPlan(task,job.method,remaining,job.samplingSeed,local,c);
    result.actualReturnTime=toc(outer); result.B=job.B; result.preparationSeconds=preparation;
    result.returnedWithinBudget=result.actualReturnTime<=job.B; result.passed=result.passed&&result.returnedWithinBudget;
    if ~result.returnedWithinBudget, result.reason='timeout'; end
    result.timeoutOverrun=max(0,result.actualReturnTime-job.B); result.job=job;
    result.codeHash=c.codeHash;
    % Call is now returned. Archival writes and independent audit are offline.
    dpWriteJson(fullfile(c.output,'active_call.json'),struct('state','returned','returnedUnix',posixtime(datetime('now','TimeZone','UTC')),'job',job));
    bundle=result.bundle; save(fullfile(folder,'trajectory.mat'),'task','result','bundle','-v7');
    % Independent research audit after return; never credited as online success.
    result.independentAuditPassed=false;
    if result.passed
        check=bdValidate(bundle,task,c,tic,120); result.independentAuditPassed=check.passed;
        result.rawMetrics=bdQuality(bundle,task,c);
        bdExportTrajectory(bundle,task,fullfile(folder,'plot_data.json'),c);
        fine=check.minClearance<.012 || max([result.rawMetrics.maxSpeedUtilization,result.rawMetrics.maxAccelerationUtilization,result.rawMetrics.maxForceUtilization])>.95 || mod(sum(double(job.runId)),10)==0;
        result.fineAuditSelected=fine;
        if fine, fineConfig=c; fineConfig.denseStep=.0025; fineReport=bdValidate(bundle,task,fineConfig,tic,120); result.fineAuditPassed=fineReport.passed; end
    end
    result.trajectoryRef=fullfile(folder,'trajectory.mat'); result=rmfield(result,{'bundle','solver'});
    if ~isempty(result.report), result.report.physical=rmDense(result.report.physical); end
    dpWriteJson(file,result); fprintf('BD_CALL %d/%d %s passed=%d %.3fs\n',k,numel(jobs),job.method,result.passed,result.actualReturnTime);
end
end
function r=rmDense(r)
for name={'q','v','a','FRequired','FInterpolated','L','Ld','Ldd','clearance','time'}, if isfield(r,name{1}), r=rmfield(r,name{1}); end; end
end
