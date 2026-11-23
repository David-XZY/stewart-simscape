function result=retargetAbfdPlanningSeed(seed,sourceProvider,targetProvider,c)
% Preserve the accepted HS motion; recompute forces for a different model.
% This creates a physically revalidated seed, never a newly optimized result.
timer=tic; model=seed.bundle.model; scene=seed.bundle.scene;
assert(strcmp(abfdHash(targetProvider.nominalModel),abfdHash(model)), ...
    'ABFD:SeedTransferModel','Target provider must use the same nominal geometry and task model.');
source=revalidateAbfdPlanningSeed(seed,model,scene,sourceProvider,c);
initial=source.initial; disc=source.disc;
oldBase=packIHSDecisionImplicit(initial.Xnode(:,2:end-1),initial.XmidIHS, ...
    initial.Anode,initial.Amid,initial.Fnode,initial.Fmid,disc,initial.separator);
assert(norm(oldBase-source.z(1:numel(oldBase)),inf)<1e-12,'ABFD:SeedTransferPacking','Source decisions differ from decoded HS data.');
for k=1:disc.numNodes
    initial.Fnode(:,k)=targetProvider.inverse(initial.Xnode(1:6,k),initial.Xnode(7:12,k),initial.Anode(:,k));
end
for k=1:disc.numMidpoints
    initial.Fmid(:,k)=targetProvider.inverse(initial.XmidIHS(1:6,k),initial.XmidIHS(7:12,k),initial.Amid(:,k));
end
base=packIHSDecisionImplicit(initial.Xnode(:,2:end-1),initial.XmidIHS, ...
    initial.Anode,initial.Amid,initial.Fnode,initial.Fmid,disc,initial.separator);
z=[base;source.z(numel(base)+1:end)];
config=c; config.designMode='sequential';
research=struct('provider',targetProvider,'config',config,'center',[],'design',[],'buildOnly',true);
if isfield(c,'plannerProfile') && any(strcmp(c.plannerProfile,{'guarded-exact-v1','guarded-adaptive-v2'}))
    research.enforceInsertionDerivatives=true;
end
nlp=buildCasadiTrackableIHSNLP(model,scene,disc,initial,struct(),research);
[cost,eq,g]=nlp.eval(z); eq=full(eq); g=full(g);
assert(all(isfinite([full(cost);eq;g])) && max(abs(eq))<=c.constraintTolerance && ...
    max(g)<=c.constraintTolerance,'ABFD:SeedTransferEquations','Transferred forces fail current physical constraints.');
traj=source.traj; traj.Unode=initial.Fnode; traj.Umid=initial.Fmid;
bundle=buildAbfdReferenceBundle(traj,model,scene,disc,targetProvider,config);
assert(isequal(bundle.refs.q,source.bundle.refs.q) && isequal(bundle.refs.qd,source.bundle.refs.qd) && ...
    isequal(bundle.refs.qdd,source.bundle.refs.qdd),'ABFD:SeedTransferMotion','Model transfer changed the accepted motion.');
result=source; result.z=z; result.initial=initial; result.traj=traj; result.bundle=bundle;
result.objective=full(cost); result.maxEq=max(abs(eq)); result.maxIneq=max([0;g]);
result.denseReport=bundle.report; result.passed=true; result.solveTime=0; result.iterations=0;
result.solverStatus='physically-revalidated-model-transfer'; result.preparationTime=toc(timer); result.totalTime=result.preparationTime;
result.implementationHash=abfdPlannerImplementationHash();
result.modelTransfer=struct('sourceProviderHash',sourceProvider.hash,'targetProviderHash',targetProvider.hash, ...
    'sourceImplementationHash',seed.implementationHash,'sourceDecisionHash',abfdHash(seed.z), ...
    'sourceSolverStatus',seed.solverStatus,'sourceIterations',seed.iterations, ...
    'method','Same HS motion and separators; target inverse dynamics; current physical equations and independent dense revalidation');
end
