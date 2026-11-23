function summary=bdMechanismDevelopment()
% Test whether one independently constructed fixed path covers the whole grid.
c=bdConfig('boundary_mechanism_screen_v1'); z=load(fullfile(c.root,'results','chapter3','boundary_geometric_probe_v1','feasible_050.mat'));
offsets=[-.02,-.01,0,.01,.02]; rows={};
for a=offsets
    for b=offsets
        task=bdMakeTask([],[a,b,0,7.5],c); [passed,r]=bdAccept(z.bundle,task,'physical',tic,30,c);
        rows{end+1}=struct('parameters',[a,b,0,7.5],'passed',passed,'reason',r.reason);
    end
end
timing={};
for duration=[7.5,3,2.4,1.92,1.536]
    task=bdMakeTask([],[0,0,0,duration],c); bundle=z.bundle; bundle.T=duration;
    [passed,r]=bdAccept(bundle,task,'physical',tic,30,c); q=bdQuality(bundle,task,c);
    bound=bdTimeLowerBound(bundle,task.model,linspace(0,1,513),c.lengthError);
    timing{end+1}=struct('T',duration,'passed',passed,'reason',r.reason,'quality',q,'timeLowerBound',bound.timeLowerBound);
end
summary=struct('gridRows',{rows},'sameFixedPathPassed',sum(cellfun(@(r)r.passed,rows)),'gridTasks',25, ...
    'timingRows',{timing},'sourcePath','independent_geometric_candidate_050_not_training_history', ...
    'mechanismEstablished',false,'formalDomainFrozen',false, ...
    'conclusion','Common-path coverage and duration diagnostics only; this is not a trained-library H1 result.');
dpWriteJson(fullfile(c.output,'summary.json'),summary);
fprintf('BD_GRID common path passed %d/25\n',summary.sameFixedPathPassed);
end
