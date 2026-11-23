function path=bdAdapt(history,task,adapterId,c)
% history carries original bundle and source task; no target reference access.
source=history.bundle; old=history.task;
if strcmp(adapterId,'A0')
    assert(strcmp(history.fitted.kind,'spline'),'BD:Representation','Missing fitted source.');
    cp=bdBaseline(task,c.controlCount); baseline=bdBaseline(old,c.controlCount);
    assert(isequal(size(cp),size(history.fitted.cp)),'BD:Representation','Representation mismatch.');
    cp(:,4:end-3)=cp(:,4:end-3)+history.fitted.cp(:,4:end-3)-baseline(:,4:end-3);
    path=struct('kind','spline','T',1,'cp',cp);
else
    assert(any(strcmp(adapterId,{'A5','A7','A5-C'})),'BD:Adapter','Unknown adapter.');
    path=struct('kind','adapted_path','adapterId',adapterId,'source',source, ...
        'sourceQ0',old.q0,'sourceGoal',old.qGoal,'q0',task.q0,'qGoal',task.qGoal);
    fcheck=bdPathFunction(path); probe=full(fcheck(linspace(0,1,257)));
    assert(all(abs(probe(5,:))<pi/2-1e-4) && max(max(abs(diff(probe(4:6,:),1,2))))<pi,'BD:RotationChart','Adapter crosses the supported Euler chart.');
    if strcmp(adapterId,'A5-C')
        f=bdPathFunction(path); s=linspace(0,1,401); Q=full(f(s));
        [B,~,~]=bdBasis(s,1,c.controlCount); cp=bdBaseline(task,c.controlCount); cp(:,4:end-3)=0;
        cp(:,4:end-3)=(Q-cp*B)/B(4:end-3,:);
        path=struct('kind','spline','T',1,'cp',cp);
    end
end
end
