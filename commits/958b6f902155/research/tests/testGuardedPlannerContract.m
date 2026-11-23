function report=testGuardedPlannerContract()
% Guards tighten the same physical NLP; the common cost and equalities stay fixed.
task=sampleTransferTask(20260906,'train'); s=task.scene; m=task.model;
d=buildTwoPhaseIHSDiscretization(s,4,2); p=buildDynamicsProvider(m);
[~,initial]=buildInitialGuessTwoPhaseIHSImplicit(m,s,d);
[initial,z]=projectAbfdHsSeed(initial,s,d,p);
c=makeAbfdConfig(); assert(~isfield(c,'plannerProfile'));
guarded=makeAbfdConfig(struct('plannerProfile','guarded-exact-v1'));
assert(~strcmp(abfdHash(c),abfdHash(guarded)));
adaptive=makeAbfdConfig(struct('plannerProfile','guarded-adaptive-v2'));
assert(~strcmp(abfdHash(guarded),abfdHash(adaptive)));
r=struct('provider',p,'config',c,'center',[],'design',[],'buildOnly',true, ...
    'enforceInsertionDerivatives',true);
plain=buildCasadiTrackableIHSNLP(m,s,d,initial,struct(),r);
r.lengthGuard=(1:6).'*1e-5; r.additionalObstacleGuard=.0005;
tight=buildCasadiTrackableIHSNLP(m,s,d,initial,struct(),r);
[ja,ea,ca]=plain.eval(z); [jb,eb,cb]=tight.eval(z);
delta=full(cb-ca); points=d.numNodes+d.numMidpoints;
assert(abs(full(ja-jb))<1e-12 && norm(full(ea-eb),inf)<1e-12);
assert(min(delta)>=-1e-12 && abs(max(delta)-.0005)<1e-12);
assert(sum(delta>1e-8)==13*points);
rejected=false;
try
    makeAbfdConfig(struct('plannerProfile','silent-fallback'));
catch
    rejected=true;
end
assert(rejected);
report=struct('passed',true,'tightenedConstraints',sum(delta>1e-8), ...
    'legacyConfigurationUnchanged',true,'scope','guard sign, unchanged equations and objective, optional profile validation');
end
