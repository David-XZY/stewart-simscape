function reportReplayCandidates()
% Offline deterministic reconstruction of raw candidates, not a new timed run.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
out=fullfile(root,'results','chapter3','payload_pair2_nofallback_v1','candidate_case_v1');
if ~isfolder(out), mkdir(out); end
c=pgConfig('payload_pair2_nofallback_v1/candidate_case_v1/replay');
c.controlCount=16; c.returnPolicy='physical';
runId='7633189c8f5ed5df37a4d091';
source=fullfile(root,'results','chapter3','payload_pair2_nofallback_v1','formal','calls',runId,'trajectory.mat');
saved=load(source,'task','result'); task=saved.task; result=saved.result;
assert(result.finalQualified && result.fineAuditPassed && strcmp(result.returnStage,'direct'));
assert(result.candidateGenerated==14 && result.selectedCandidateId==13);
modelFile=fullfile(root,'results','chapter3','payload_formal_models_v1',result.job.modelKey,'model.json');
asset.model=jsondecode(fileread(modelFile)); asset.callContext=pgEncodeContext(task,'basic');
pgModelCheck(asset.model,result.method,c);
[pool,~]=pgCandidates(result.method,task,1:14,asset,tic,inf,c,result.samplingSeed);
assert(numel(pool)==14 && all(cellfun(@(b)~isempty(b.bundle),pool)));
controlError=max(abs(pool{13}.bundle.cp(:)-result.bundle.cp(:)));
assert(controlError<1e-12,'Replay:Mismatch','Replayed accepted candidate differs from the saved original.');
t=linspace(0,task.T,ceil(task.T/.0025)+1);
candidateQ=zeros(6,numel(t),14); candidateCP=zeros(6,16,14);
checks=cell(13,1);
for k=1:14
    tr=bdReconstruct(pool{k}.bundle,t); candidateQ(:,:,k)=tr.q; candidateCP(:,:,k)=pool{k}.bundle.cp;
    if k<=13
        check=pgValidate(pool{k}.bundle,task,c);
        checks{k}=struct('id',k,'passed',check.passed,'reason',check.reason);
        assert(check.passed==(k==13),'Replay:Acceptance','Raw candidate acceptance differs.');
    end
end
fineConfig=c; fineConfig.denseStep=.0025;
fine=pgValidate(result.bundle,task,fineConfig); assert(fine.passed);
tr=bdReconstruct(result.bundle,t); acceptedQ=tr.q;
q0=task.q0; qGoal=task.qGoal; obstacles=task.scene.hood.obstacles;
metadata=struct('runId',runId,'taskId',task.taskId,'method',result.method, ...
    'trainingSeed',result.trainingSeed,'samplingSeed',result.samplingSeed,'B',result.B,'T',task.T, ...
    'candidateGenerated',14,'originalCheckedRawCandidates',13,'selectedCandidateId',13, ...
    'unexaminedCandidateId',14,'originalActualReturnTime',result.actualReturnTime, ...
    'maxControlPointReplayError',controlError,'independentReplayPassed',true,'fineReplayPassed',fine.passed, ...
    'minimumClearanceM',fine.minClearance,'minimumSigma',fine.minSigma,'maximumCondition',fine.maxCondition, ...
    'rawMetrics',result.rawMetrics,'checks',{checks},'matlabReplayVersion',version, ...
    'scope','Raw candidates reconstructed offline from original fixed model and candidate RNG; accepted candidate agrees with original saved controls. Original timings remain unchanged. Candidate 14 was generated but not checked in the original call. Earlier repair trajectories are not displayed.');
save(fullfile(out,'candidate_plot_data.mat'),'candidateQ','candidateCP','acceptedQ','q0','qGoal','obstacles','t','metadata','-v7');
dpWriteJson(fullfile(out,'replay_evidence.json'),metadata);
fprintf('REPLAY_OK controls_max_error=%.17g minClearance=%.9g fine=%d\n',controlError,fine.minClearance,fine.passed);
end
