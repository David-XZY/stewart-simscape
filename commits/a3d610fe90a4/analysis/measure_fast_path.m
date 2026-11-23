function rows=measure_fast_path(runName,worker,workers)
% Frozen diagnostic: direct dense acceptance, existing bounded projection,
% then the original IHSID. Additional candidate census is outside online time.
c=setupDiffusion(); source=fullfile(c.output,runName);
out=fullfile(c.output,[runName,'_fastpath_v2']); if ~isfolder(out), mkdir(out); end
modelFile=fullfile(source,'training','model.json');
files=dir(fullfile(source,'benchmark','*','diffusion','benchmark.mat'));
assert(numel(files)==20,'Expected the frozen 20-task formal test set.');
protocol=struct('version',1,'sourceRun',runName,'candidateMethod','unguided_diffusion', ...
    'order','original normalized violation then smoothness', ...
    'direct','ranked dense checks, stop on first pass', ...
    'repair','existing dpQuickFit, only if all eight direct checks fail, stop on first pass', ...
    'repairMaxIterations',40,'repairChangeRmsLimit',.25, ...
    'fallback','original refineAndValidate, at most eight candidates', ...
    'extraCensus','remaining direct candidates checked after online timer stops', ...
    'implementationHash',dpImplementationHash(c),'measurementHash',dpHash(fileread([mfilename('fullpath'),'.m'])), ...
    'modelHash',dpHash(fileread(modelFile)),'config',dpRunConfig(c));
rows=[];
for index=worker:workers:numel(files)
    s=load(fullfile(files(index).folder,files(index).name)); task=s.task;
    folder=fullfile(out,task.taskId); if ~isfolder(folder), mkdir(folder); end
    file=fullfile(folder,'measurement.mat'); fingerprint=dpHash(struct('protocol',protocol,'task',task.hash));
    if isfile(file)
        old=load(file,'fingerprint','row'); assert(strcmp(old.fingerprint,fingerprint),'Changed measurement fingerprint.'); row=old.row;
    else
        clear dpSolve;
        direct=struct('candidate',{},'rank',{},'passed',{},'seconds',{},'report',{},'reason',{});
        repairs=struct('candidate',{},'rank',{},'passed',{},'seconds',{},'report',{},'info',{},'cp',{},'reason',{});
        fallback=[]; finalReport=[]; acceptedCp=[]; chosen=NaN; stage='failed'; reason='';
        online=tic; st=tic;
        [candidates,generation]=sampleCandidates(task,modelFile,8,c.seed+task.seed,false,c);
        samplingSeconds=toc(st);
        assert(isequal(candidates,s.candidates),'Fresh candidates differ from frozen baseline.');
        assert(isequal(generation.initialNoise,s.generation.initialNoise),'Noise differs.');
        st=tic; [scores,order]=rankCandidates(task,candidates); rankingSeconds=toc(st);
        assert(isequal(order,s.outcome.order),'Ranking differs from frozen baseline.');
        assert(max(abs(scores(:)-s.outcome.scores(:)))<1e-8,'Ranking scores differ.');
        directTimer=tic;
        for rank=1:8
            k=order(rank); entry=checkCandidate(candidates(:,:,k),task,c,k,rank);
            direct(end+1)=entry; %#ok<AGROW>
            if entry.passed
                stage='direct'; chosen=k; acceptedCp=candidates(:,:,k); finalReport=entry.report; break;
            end
        end
        directSeconds=toc(directTimer); directOnlineCount=numel(direct);
        repairSeconds=0; fallbackSeconds=0;
        if ~strcmp(stage,'direct')
            repairTimer=tic;
            for rank=1:8
                k=order(rank); st=tic; cp=candidates(:,:,k); report=[]; info=[]; err='';
                try
                    [cp,report,info]=dpQuickFit(task,cp,c); passed=report.passed;
                catch e, passed=false; err=[e.identifier,': ',e.message]; end
                entry=struct('candidate',k,'rank',rank,'passed',passed,'seconds',toc(st), ...
                    'report',report,'info',info,'cp',cp,'reason',err);
                repairs(end+1)=entry; %#ok<AGROW>
                if passed, stage='repair'; chosen=k; acceptedCp=cp; finalReport=report; break; end
            end
            repairSeconds=toc(repairTimer);
        end
        if strcmp(stage,'failed')
            st=tic; local=c; local.output=fullfile(folder,'fallback');
            try
                fallback=refineAndValidate(task,candidates,local);
                if fallback.passed, stage='ihsid'; finalReport=fallback.best.report; chosen=fallback.attempts(end).candidate;
                else, reason='All original IHSID candidates failed.'; end
            catch e, reason=[e.identifier,': ',e.message]; end
            fallbackSeconds=toc(st);
        end
        onlineSeconds=toc(online);
        % Census all eight originals, without charging diagnostics to planning.
        for rank=directOnlineCount+1:8
            k=order(rank); direct(end+1)=checkCandidate(candidates(:,:,k),task,c,k,rank); %#ok<AGROW>
        end
        assert(numel(direct)==8);
        finalPassed=~strcmp(stage,'failed'); if finalPassed, assert(finalReport.passed); end
        fallbackAttempts=0; if ~isempty(fallback), fallbackAttempts=numel(fallback.attempts); end
        row=struct('taskId',task.taskId,'family',task.family,'directCandidatePasses',sum([direct.passed]), ...
            'directTaskPassed',strcmp(stage,'direct'),'repairAttempted',~isempty(repairs), ...
            'repairTaskPassed',strcmp(stage,'repair'),'needsIhsid',~any(strcmp(stage,{'direct','repair'})), ...
            'finalPassed',finalPassed,'acceptedStage',stage,'chosenCandidate',chosen, ...
            'samplingSeconds',samplingSeconds,'rankingSeconds',rankingSeconds, ...
            'directOnlineCount',directOnlineCount,'directSeconds',directSeconds, ...
            'repairAttempts',numel(repairs),'repairSeconds',repairSeconds, ...
            'fallbackAttempts',fallbackAttempts,'fallbackSeconds',fallbackSeconds, ...
            'onlineSeconds',onlineSeconds,'oldOnlineSeconds',s.row.seconds, ...
            'censusSeconds',sum([direct.seconds]),'reason',reason);
        save(file,'row','protocol','fingerprint','task','candidates','generation','scores','order', ...
            'direct','repairs','fallback','acceptedCp','finalReport','-v7');
        dpWriteJson(fullfile(folder,'summary.json'),row);
    end
    if isempty(rows), rows=row; else, rows(end+1)=row; end %#ok<AGROW>
    dpWriteJson(fullfile(out,sprintf('worker_%d.json',worker)),rows);
    writetable(struct2table(rows,'AsArray',true),fullfile(out,sprintf('worker_%d.csv',worker)));
    fprintf('FASTPATH %s stage=%s direct=%d/8 repairAttempts=%d online=%.2f old=%.2f\n', ...
        row.taskId,row.acceptedStage,row.directCandidatePasses,row.repairAttempts,row.onlineSeconds,row.oldOnlineSeconds);
end
end

function e=checkCandidate(cp,task,c,k,rank)
timer=tic; report=[]; reason='';
try
    report=dpValidate(struct('kind','spline','cp',cp,'T',task.T),task,c); passed=report.passed;
catch ex, passed=false; reason=[ex.identifier,': ',ex.message]; end
e=struct('candidate',k,'rank',rank,'passed',passed,'seconds',toc(timer),'report',report,'reason',reason);
end

function [scores,order]=rankCandidates(task,candidates)
% Exact original ranking; keep production code and frozen hash unchanged.
scores=inf(8,2); p=dpPhysics(task.model); m=task.model;
for k=1:8
    try
        tr=dpSpline(candidates(:,:,k),linspace(0,task.T,61),task.T);
        [L,Ld,Ldd,F]=p.point(tr.q,tr.v,tr.a); L=full(L); Ld=full(Ld); Ldd=full(Ldd); F=full(F);
        d=zeros(1,61); for i=1:61, ds=dpClearance(tr.q(:,i),task,false); d(i)=min(ds); end
        residual=[max(0,(m.lmin-L)/.1);max(0,(L-m.lmax)/.1);max(0,abs(Ld)/.6-1); ...
            max(0,abs(Ldd)/1.2-1);max(0,abs(F)/2000-1);max(0,(.01-d)/.01)];
        a=tr.a./repmat([.1;.1;.1;deg2rad(10)*ones(3,1)],1,size(tr.a,2));
        scores(k,:)=[sum(residual(:).^2),sum(a(:).^2)];
    catch, scores(k,:)=[inf,inf]; end
end
[~,order]=sortrows(scores,[1,2]);
end
