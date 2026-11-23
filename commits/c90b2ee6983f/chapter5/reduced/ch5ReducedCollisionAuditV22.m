function certificate=ch5ReducedCollisionAuditV22(trace,o,r)
% One generic obstacle kernel per fixed time grid; active masks are data.
persistent caches
if isempty(caches),caches=containers.Map('KeyType','char','ValueType','any');end
times=reshape(r.plant.traceTime(:)+(0:r.c.H-1)*r.c.Ts,1,[]);key=char(r.c.version+"|"+mat2str(times));
if ~isKey(caches,key)
    import casadi.*
    nq=numel(times);Q=MX.sym('collision_trace_generic',6,nq);O=MX.sym('collision_obstacle_generic',22);
    future=repmat(O,1,nq);future(1:3,:)=repmat(O(1:3),1,nq)+O(16:18)*times;
    mapped=r.collision.single.map(nq);gaps=mapped(Q,future);
    caches(key)=Function('ch5_collision_generic_audit',{Q,O},{mmin(gaps)},ch5JitOptions());
end
% Build even at empty observation, so prepare can warm without future events.
certificate=inf;audit=caches(key);
for obstacle=find(o(22,:)>.5)
    certificate=min(certificate,full(audit(trace(1:6,:),o(:,obstacle))));
end
end
