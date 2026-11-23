function result=ch5PlanPostcheck(plan,o,r)
times=0:r.c.internal_step:plan.t(end);minimum=inf;worst=0;
for t=times
    [q,v,a]=ch5Reference(plan,t);[d,G,~,~,~,Jq,~,L,Ld,bias]=r.ad.dynamics(q,v);
    u=full(G)\(a-full(d));[sigma]=r.ad.singularity(q);ot=o;ot(1:3,:)=o(1:3,:)+t*o(16:18,:);gap=full(r.collision.value(q,ot));
    acc=full(bias+Jq*a);
    margins=[full(L)-r.physical.lmin;r.physical.lmax-full(L);r.physical.actuator.ldotMax-abs(full(Ld)); ...
        r.physical.actuator.lddotMax-abs(acc);full(sigma)-r.strict.sigmaSafe^2;gap-r.c.safe_distance; ...
        u-r.strict.forceMin;r.strict.forceMax-u];
    minimum=min(minimum,min(margins));worst=max(worst,-min(margins));
end
result=struct('accepted',minimum>=-1e-5,'minimum_margin',minimum,'maximum_violation',worst,'samples',numel(times));
end
