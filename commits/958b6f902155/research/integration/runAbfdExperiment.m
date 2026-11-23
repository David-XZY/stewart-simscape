function result=runAbfdExperiment(task,c)
c=makeAbfdConfig(c);
root=setupAbfdResearch(); setupCasadiIpoptMa27(root);
profile=makeHighFidelityPlantProfile(c.plantProfile,c.plantOverrides);
[p,c]=buildAbfdProviderForTask(task.model,c,profile,task.scene.q0);
fingerprint=abfdRunFingerprint(task,c);
exportExperimentManifest(c.outputDir,c,'running',struct('taskId',task.taskId,'providerHash',p.hash,'fingerprint',fingerprint));
try
    if strcmp(c.plannerMode,'learned-warmstart-F')
        candidates=sampleAbfdCandidates(task,c.proposalFile,c.candidateCount,c.seed);
        [initial,ranking]=rankAbfdProposalCandidates(task,candidates,p,c);
        scores=ranking.scores; order=ranking.order; reasons=ranking.reasons;
        best=[]; candidatesLog=cell(1,c.refineCount);
        save(fullfile(c.outputDir,'candidate_ranking.mat'),'ranking','candidates');
        for j=1:numel(ranking.selected)
            k=ranking.selected(j);
            try
                candidateConfig=c; candidateConfig.outputDir=fullfile(c.outputDir,sprintf('candidate_%02d',k));
                planning=plan(task,p,candidateConfig,initial{k}); candidatesLog{j}=planning;
                if planning.passed
                    merit=planning.objective;
                    if isfield(planning,'finalMerit'), merit=planning.finalMerit; end
                    planning.selectionMerit=merit;
                    if isempty(best) || merit<best.selectionMerit, best=planning; end
                end
            catch e, reasons{k}=e.message;
            end
        end
        save(fullfile(c.outputDir,'candidate_audit.mat'),'scores','order','reasons','candidatesLog');
        assert(~isempty(best),'ABFD:NoFeasibleCandidate','All refined proposals failed.'); planning=best;
    elseif ~isempty(c.trajectoryFile)
        assert(~strcmp(c.designMode,'codesign-A'),'ABFD:FixedReferenceCodesign', ...
            'A requires trajectory/gain optimization; a fixed-reference bypass is not codesign acceptance.');
        data=load(c.trajectoryFile,'bundle');
        assert(isfield(data,'bundle'),'ABFD:MissingBundle','Research bundle required; legacy node-only file is not accepted.');
        bundle=data.bundle;
        % Fixed trajectory B ablation: same q/v/a, selected-model force recomputation.
        bundle=buildAbfdReferenceBundle(bundle.refs.hs,task.model,task.scene,bundle.disc,p,c);
        planning=struct('passed',true,'bundle',bundle,'objective',NaN);
    else, planning=plan(task,p,c,[]);
    end
    save(fullfile(c.outputDir,'planning_diagnostics.mat'),'planning');
    assert(planning.passed,'ABFD:PlanningRejected','Planner did not pass dense acceptance.');
    bundle=planning.bundle; save(fullfile(c.outputDir,'reference_bundle.mat'),'bundle');
    profile=makeHighFidelityPlantProfile(c.plantProfile,c.plantOverrides);
    result=runAbfdSimscapeCase(bundle,p,c,profile); result.planning=planning;
    result.log.taskId=task.taskId; result.log.loadId=task.loadId; result.log.trajectoryId=[task.taskId,'_',c.designMode];
    save(fullfile(c.outputDir,'result.mat'),'result');
    outputHashes=struct('result',abfdFileHash(fullfile(c.outputDir,'result.mat')), ...
        'reference',abfdFileHash(fullfile(c.outputDir,'reference_bundle.mat')));
    exportExperimentManifest(c.outputDir,c,'experiment-completed',struct('metrics',result.metrics, ...
        'passed',result.passed,'acceptance',result.acceptance,'outputHashes',outputHashes,'fingerprint',fingerprint));
catch e
    exportExperimentManifest(c.outputDir,c,'failed',struct('identifier',e.identifier,'message',e.message,'fingerprint',fingerprint)); rethrow(e);
end
end
function result=plan(task,p,c,initial)
if strcmp(c.designMode,'codesign-A'), result=run_A1_alternating_codesign(task.model,task.scene,p,c,initial);
else, result=solveAbfdIhsid(task.model,task.scene,p,c,initial); end
end
