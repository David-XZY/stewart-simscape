function info=pgBuildProblem(c)
% Four resident structures: two declared shapes, open / two-fixture scenes.
started=tic; rows={};
for shape={'cylinder','box'}
    for obstacles=[0,2]
        base=bdMakeTask([],[0,0,0,7.5],c);
        % Fixed public prototype only. No test task endpoint/pose is preloaded.
        if obstacles==0, base.scene.hood.obstacles=base.scene.hood.obstacles([]); end
        if strcmp(shape{1},'cylinder'), dims=[1,.12]; else, dims=[1,.24,.20]; end
        task=pgFinalizeTask(base,pgMakePayload(shape{1},dims,100),c); cc=c; cc.buildOnly=true; cc.clockHandle=tic;
        cp=bdBaseline(task,c.controlCount); result=pgSolve(task,cp,cc); result.shape=shape{1}; result.obstacleCount=obstacles;
        penaltyClock=tic; pgPenalty(reshape(cp(:,4:end-3),[],1),task,c); result.penaltyWarmupSeconds=toc(penaltyClock); rows{end+1}=result;
    end
end
info=struct('structures',{rows},'seconds',toc(started),'trajectorySolveCalls',0);
end
