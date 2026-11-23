function result=revalidateAbfdPlanningSeed(seed,model,scene,provider,c)
% Reuse a completed optimizer solution only after current equations and dense
% physical constraints pass. Historical solver success alone is insufficient.
timer=tic;
required={'passed','bundle','z','initial','disc','objective','implementationHash'};
assert(all(isfield(seed,required)) && seed.passed && seed.bundle.publishPassed, ...
    'ABFD:UnacceptedPlanningSeed','A previously accepted complete planning result is required.');
assert(strcmp(seed.bundle.providerHash,provider.hash) && ...
    strcmp(abfdHash(seed.bundle.model),abfdHash(model)) && ...
    strcmp(abfdHash(seed.bundle.scene),abfdHash(scene)), ...
    'ABFD:PlanningSeedMismatch','Seed model, dynamics provider or scene differs.');
disc=buildTwoPhaseIHSDiscretization(scene,scene.phase.numIntervalsApproach,scene.phase.numIntervalsInsertion);
assert(strcmp(abfdHash(seed.disc),abfdHash(disc)), ...
    'ABFD:PlanningSeedDiscretization','Seed discretization differs.');
base=c; base.designMode='sequential';
research=struct('provider',provider,'config',base,'center',[],'design',[],'buildOnly',true);
if isfield(c,'plannerProfile') && any(strcmp(c.plannerProfile,{'guarded-exact-v1','guarded-adaptive-v2'}))
    research.enforceInsertionDerivatives=true;
end
% Original physical scene: numerical planning guards are not relaxed physical
% limits and need not be carried from the historical solver into this check.
nlp=buildCasadiTrackableIHSNLP(model,scene,disc,seed.initial,struct(),research);
validateattributes(seed.z,{'double'},{'column','numel',numel(nlp.z),'real','finite'});
[cost,equality,inequality]=nlp.eval(seed.z); equality=full(equality); inequality=full(inequality);
maxEq=max(abs(equality)); maxIneq=max([0;inequality]);
assert(isfinite(full(cost)) && all(isfinite([equality;inequality])) && ...
    maxEq<=c.constraintTolerance && maxIneq<=c.constraintTolerance, ...
    'ABFD:PlanningSeedEquations','Seed fails current equations/inequalities: %g / %g.',maxEq,maxIneq);
[X,Xm,An,Am,Fn,Fm,~,separator]=unpackIHSDecisionImplicit(seed.z(1:nlp.baseNumZ),scene,disc);
traj=struct('t',disc.tNode,'tc',disc.tMid,'Xnode',X,'Xmid',Xm, ...
    'Anode',An,'Amid',Am,'Unode',Fn,'Umid',Fm);
refs=reconstructIhsidContinuousReference(traj,provider,c.sampleTime);
dense=validateAbfdReference(refs,model,scene,c);
assert(dense.passed,'ABFD:PlanningSeedDense','Reused seed fails current independent dense acceptance.');
result=seed; result.objective=full(cost); result.maxEq=maxEq; result.maxIneq=maxIneq;
result.traj=traj; result.disc=disc; result.denseReport=dense;
result.bundle=buildAbfdReferenceBundle(traj,model,scene,disc,provider,base);
result.initial.Xnode=X; result.initial.XmidIHS=Xm; result.initial.Anode=An; result.initial.Amid=Am;
result.initial.Fnode=Fn; result.initial.Fmid=Fm; result.initial.separator=separator;
result.seedReuse=struct('kind','revalidated completed sequential solution', ...
    'sourceImplementationHash',seed.implementationHash,'sourceObjective',seed.objective, ...
    'validationImplementationHash',abfdPlannerImplementationHash(),'validationSeconds',toc(timer));
if isfield(seed,'modelTransfer'), result.seedReuse.kind='revalidated transferred physical seed'; end
end
