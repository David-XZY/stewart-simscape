function diagnose_separator()
c=setupDiffusion(); task=makeTask('simple',20260915,'development',true); result=dpSolve(task,dpSeed(task,1),c);
save(fullfile(c.output,'separator_probe.mat'),'task','result','-v7'); b=result.bundle;
for k=1:size(b.X,2)
    q=b.X(1:6,k); d=dpClearance(q,task); n=b.separators(1:3,k);
    p=dpPhysics(task.model); value=full(dpGapExpr(q,n,task.scene.hood.obstacles(1),task,p));
    if d<.009
        fprintf('Point %d independent %.9f NLPnormal %.9f norm %.9f\n',k,d,value,norm(n)); disp(n.');
        disp(b.separators(:,k).'); break;
    end
end
end
