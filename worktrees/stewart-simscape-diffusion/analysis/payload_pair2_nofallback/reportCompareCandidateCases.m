function reportCompareCandidateCases()
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
base=fullfile(root,'results','chapter3','payload_pair2_nofallback_v1');
out=fullfile(base,'candidate_joint_cases_v1'); if ~isfolder(out), mkdir(out); end
c=pgConfig('payload_pair2_nofallback_v1/candidate_joint_cases_v1/replay'); c.controlCount=16; c.returnPolicy='physical';
ids={'7633189c8f5ed5df37a4d091','71c93079c72cff94b9929aed','5614f8d16b94d281eb94c661','9743ce8900e3fdbac6538d7e','a3e60ba1435daf54061fb10d','09b4725ac0c47cf6ed38babe'};
summaries=cell(size(ids));
for i=1:numel(ids)
    source=fullfile(base,'formal','calls',ids{i},'trajectory.mat'); d=load(source,'task','result'); task=d.task; result=d.result;
    assert(result.finalQualified); n=result.candidateGenerated;
    asset.model=jsondecode(fileread(fullfile(root,'results','chapter3','payload_formal_models_v1',result.job.modelKey,'model.json')));
    asset.callContext=pgEncodeContext(task,'basic');
    if contains(result.method,'signature'), asset.callContext=[asset.callContext;pgPhysicsSignature(task)]; end
    pgModelCheck(asset.model,result.method,c);
    [pool,~]=pgCandidates(result.method,task,1:n,asset,tic,inf,c,result.samplingSeed);
    assert(numel(pool)==n && all(cellfun(@(x)~isempty(x.bundle),pool)));
    t=linspace(0,task.T,801); candidateQ=zeros(6,numel(t),n); candidateCP=zeros(6,16,n);
    for k=1:n
        tr=bdReconstruct(pool{k}.bundle,t); candidateQ(:,:,k)=tr.q; candidateCP(:,:,k)=pool{k}.bundle.cp;
    end
    pairRms=[]; pairPeak=[];
    for k=1:n
        for j=k+1:n
            delta=candidateQ(1:3,:,k)-candidateQ(1:3,:,j); dist=sqrt(sum(delta.^2,1));
            pairRms(end+1)=sqrt(mean(dist.^2)); pairPeak(end+1)=max(dist);
        end
    end
    tr=bdReconstruct(result.bundle,t); acceptedQ=tr.q; q0=task.q0; qGoal=task.qGoal; obstacles=task.scene.hood.obstacles;
    s=result.selectedCandidateId; raw=pool{s}.bundle;
    if strcmp(result.returnStage,'repair'), [repaired,repairInfo]=p2Repair(raw,task,tic,inf,c); else, repaired=raw; repairInfo=[]; end
    replayError=max(abs(repaired.cp(:)-result.bundle.cp(:)));
    assert(replayError<1e-10,'Replay:Mismatch','Selected candidate reconstruction mismatch.');
    fineC=c; fineC.denseStep=.0025; fine=pgValidate(result.bundle,task,fineC); assert(fine.passed);
    angle=acos(max(-1,min(1,(trace(rpy2rotmZYX(qGoal(4:6))*rpy2rotmZYX(q0(4:6)).')-1)/2)))*180/pi;
    metadata=struct('runId',ids{i},'taskId',task.taskId,'method',result.method,'B',result.B,'T',task.T, ...
        'generated',n,'selectedId',s,'returnStage',result.returnStage,'originalReturnTime',result.actualReturnTime, ...
        'q0',q0,'qGoal',qGoal,'endpointTranslation',norm(qGoal(1:3)-q0(1:3)),'endpointRotationDeg',angle, ...
        'medianPairRmsM',median(pairRms),'maxPairRmsM',max(pairRms),'maxPairSeparationM',max(pairPeak), ...
        'replaySelectedError',replayError,'finePassed',fine.passed,'minClearanceM',fine.minClearance, ...
        'rawMetrics',result.rawMetrics,'originalCandidateEvents',{result.candidateEvents},'originalAttempts',{result.attempts}, ...
        'repairReplay',repairInfo,'modelKey',result.job.modelKey,'seed',result.samplingSeed, ...
        'scope','Raw candidates deterministically replayed. Final trajectory and timings from original saved run; original acceptance stage preserved. Pair distances use corresponding normalized-time samples, not a proof of geometric modes.');
    folder=fullfile(out,ids{i}); if ~isfolder(folder), mkdir(folder); end
    save(fullfile(folder,'plot_data.mat'),'candidateQ','candidateCP','acceptedQ','q0','qGoal','obstacles','t','metadata','-v7');
    dpWriteJson(fullfile(folder,'evidence.json'),metadata); summaries{i}=rmfield(metadata,{'originalCandidateEvents','originalAttempts','rawMetrics','repairReplay'});
    fprintf('CASE %s n=%d angle=%.1f peak=%.4f medianRMS=%.4f replay=%.3g\n',ids{i},n,angle,max(pairPeak),median(pairRms),replayError);
end
dpWriteJson(fullfile(out,'comparison.json'),summaries);
end
