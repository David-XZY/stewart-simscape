function result=ch5SpeedPlanPostcheck(plan,o,r)
% Batch native evaluation of exactly the original full dense physical gate.
persistent maps
if isempty(maps),maps=containers.Map('KeyType','char','ValueType','any');end
times=0:r.c.internal_step:plan.t(end);[q,v,a]=ch5SpeedReferenceBatch(plan,times);n=numel(times);
key=char(r.c.version);
if ~isKey(maps,key)
    import casadi.*
    O=MX.sym('dense_o',22,r.c.max_obstacles);T=MX.sym('dense_time');
    oneQ=MX.sym('one_q',6);oneV=MX.sym('one_v',6);oneA=MX.sym('one_a',6);
    F=r.ad.feedforward(oneQ,oneV,oneA);[~,value]=r.ad.implicit(oneQ,oneV,oneA,F);
    gaps={};for j=1:r.c.max_obstacles
        future=MX(O(:,j));future(1:3)=O(1:3,j)+O(16:18,j)*T;
        gaps{j}=r.collision.single(oneQ,future);
    end
    margins=[value(1:6)-r.physical.lmin;r.physical.lmax-value(1:6); ...
        r.physical.actuator.ldotMax-fabs(value(7:12));r.physical.actuator.lddotMax-fabs(value(13:18)); ...
        value(19)-r.strict.sigmaSafe^2;vertcat(gaps{:})-r.c.safe_distance; ...
        value(20:25)-r.strict.forceMin;r.strict.forceMax-value(20:25)];
    maps(key)=Function('dense_check',{oneQ,oneV,oneA,O,T},{mmin(margins)},ch5ProjectionJitOptions());
end
single=maps(key);mapped=single.map(n);value=min(full(mapped(q,v,a,repmat(o,1,n),times)));
result=struct('accepted',value>=-1e-5,'minimum_margin',value,'maximum_violation',max(0,-value),'samples',n);
end
