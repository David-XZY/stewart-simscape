function map=ch5IntervalConstraints(r,active)
% Same full nonideal interval constraints, reused across all shooting stages.
import casadi.*
assert(r.mode=="nonideal-force");
xx=MX.sym('interval_x',r.plant.n);uu=MX.sym('interval_u',6);
O=MX.sym('interval_obstacles',22,r.c.max_obstacles);start=MX.sym('interval_start');
fd=ch5DerivativeMap(r);[~,states]=fd(xx,uu);blocks={};
for k=1:numel(r.plant.traceTime)
    tx=states(:,k);ot=MX(O);
    ot(1:3,:)=O(1:3,:)+(start+r.plant.traceTime(k))*O(16:18,:);
    value=r.plant.mechanicalValues(tx,uu);gap=MX.zeros(numel(active),1);
    for oi=1:numel(active),gap(oi)=r.collision.single(tx(1:6),ot(:,active(oi)));end
    blocks{end+1}=[value(1:19);gap]; %#ok<AGROW>
end
map=Function('ch5_interval_constraints',{xx,uu,O,start},{vertcat(blocks{:})},struct('max_num_dir',4));
end
