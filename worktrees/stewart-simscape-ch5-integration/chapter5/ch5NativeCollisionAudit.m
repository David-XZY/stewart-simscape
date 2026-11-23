function certificate=ch5NativeCollisionAudit(trace,o,r)
% Same separating-plane certificate and CV prediction on the complete inner
% grid, evaluated in one native call instead of one MATLAB call per point.
persistent caches
if isempty(caches),caches=containers.Map('KeyType','char','ValueType','any');end
active=find(o(22,:)>.5);
if isempty(active),certificate=inf;return;end
times=reshape(r.plant.traceTime(:)+(0:r.c.H-1)*r.c.Ts,1,[]);
key=char(r.c.version+"|"+mat2str(active)+"|"+mat2str(times));
if ~isKey(caches,key)
    import casadi.*
    nq=numel(times);Q=MX.sym('collision_trace',6,nq);O=MX.sym('collision_obstacles',22,r.c.max_obstacles);
    mapped=r.collision.single.map(nq);gaps=cell(numel(active),1);
    for j=1:numel(active)
        future=MX(repmat(O(:,active(j)),1,nq));
        future(1:3,:)=repmat(O(1:3,active(j)),1,nq)+O(16:18,active(j))*times;
        gaps{j}=reshape(mapped(Q,future),nq,1);
    end
    caches(key)=Function('ch5_collision_audit',{Q,O},{mmin(vertcat(gaps{:}))},ch5JitOptions());
end
audit=caches(key);certificate=full(audit(trace(1:6,:),o));
end
