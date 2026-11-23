function [initial,audit]=rankAbfdProposalCandidates(task,candidates,p,c)
% Shared ranking for the numerical F benchmark and the Simscape experiment.
timer=tic; count=size(candidates,3);
assert(size(candidates,1)==6 && size(candidates,2)==10 && count==c.candidateCount, ...
    'ABFD:ProposalShape','Expected the configured number of 6-by-10 candidates.');
disc=buildTwoPhaseIHSDiscretization(task.scene,task.scene.phase.numIntervalsApproach,task.scene.phase.numIntervalsInsertion);
scores=inf(1,count); initial=cell(size(scores)); reasons=cell(size(scores));
for k=1:count
    try
        [~,initial{k}]=buildLearnedInitialGuessIHSID(candidates(:,:,k),task.model,task.scene,disc,p);
        if isfield(c,'plannerProfile') && any(strcmp(c.plannerProfile,{'guarded-exact-v1','guarded-adaptive-v2'}))
            initial{k}=projectAbfdHsSeed(initial{k},task.scene,disc,p);
        end
        raw=initial{k}; tr=struct('t',disc.tNode,'tc',disc.tMid,'Xnode',raw.Xnode, ...
            'Xmid',raw.XmidIHS,'Anode',raw.Anode,'Amid',raw.Amid);
        refs=reconstructIhsidContinuousReference(tr,p,disc.h/2);
        scoring=c; scoring.controllerMode='geometric-D';
        metric=run_A0_trackability_metric(refs,p,scoring);
        assert(isscalar(metric.predictedCost) && isfinite(metric.predictedCost), ...
            'ABFD:ProposalScore','Nonfinite candidate score.');
        scores(k)=metric.predictedCost;
    catch e
        reasons{k}=e.message;
    end
end
[~,order]=sort(scores); selected=order(1:min(c.refineCount,count));
selected=selected(isfinite(scores(selected)));
audit=struct('scores',scores,'order',order,'selected',selected,'reasons',{reasons}, ...
    'seconds',toc(timer),'diversity',evaluateProposalDiversity(candidates), ...
    'scope','Predicted trackability ranking only; selected candidates require independent optimizer and dense acceptance.');
end
