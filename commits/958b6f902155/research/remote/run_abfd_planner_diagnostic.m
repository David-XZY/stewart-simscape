function report=run_abfd_planner_diagnostic(kind,index,checkpoint,outputDir,hessian,scaled,barrier,anchor,trustRadius,lengthGuard,numerics)
% Compare solver curvature on the unchanged task, constraints and objective.
if nargin<6, scaled=false; end
if nargin<7, barrier='adaptive'; end
if nargin<8, anchor=false; end
if nargin<9, trustRadius=inf; end
if nargin<10, lengthGuard=0; end
if nargin<11, numerics=struct(); end
validateattributes(trustRadius,{'double'},{'scalar','positive','nonnan'});
validateattributes(lengthGuard,{'double'},{'scalar','nonnegative','finite'});
validatestring(barrier,{'adaptive','monotone'});
import casadi.*
root=setupAbfdResearch(); setupCasadiIpoptMa27(root);
if ~isfolder(outputDir), mkdir(outputDir); end
tasks=makeAbfdTaskManifest(kind,60,20260905); task=tasks(index);
solverScene=task.scene;
if isfield(numerics,'collisionSmoothing')
    validateattributes(numerics.collisionSmoothing,{'double'},{'scalar','positive','finite'});
    solverScene.collision.smoothingEps=numerics.collisionSmoothing;
end
p=buildDynamicsProvider(task.model);
c=makeAbfdConfig(struct('solverMaxIterations',1500,'solverMaxCpuTime',600,'outputDir',outputDir));
disc=buildTwoPhaseIHSDiscretization(task.scene,task.scene.phase.numIntervalsApproach,task.scene.phase.numIntervalsInsertion);
if isempty(checkpoint)
    checkpointKind='nominal';
    [~,initial]=buildInitialGuessTwoPhaseIHSImplicit(task.model,task.scene,disc);
    z=packIHSDecisionImplicit(initial.Xnode(:,2:end-1),initial.XmidIHS,initial.Anode,initial.Amid,initial.Fnode,initial.Fmid,disc,initial.separator);
    [~,extra]=buildAbfdSeparators(initial.Xnode(1:6,:),initial.XmidIHS(1:6,:),task.scene,disc); z=[z;extra(:)];
else
    saved=load(checkpoint);
    if isfield(saved,'initial') && isfield(saved,'z')
        initial=saved.initial; z=saved.z; checkpointKind='projected-HS';
    elseif isfield(saved,'cp')
        [z,initial]=buildLearnedInitialGuessIHSID(saved.cp,task.model,task.scene,disc,p);
        checkpointKind='spline';
    else
        if isfield(saved,'planning'), old=saved.planning; else, old=saved.result; end
        initial=old.initial; z=old.z;
        checkpointKind='previous-NLP';
    end
end
if isfield(numerics,'collisionSmoothing')
    [initial.separator,extra]=buildAbfdSeparators(initial.Xnode(1:6,:),initial.XmidIHS(1:6,:),solverScene,disc);
    z=packIHSDecisionImplicit(initial.Xnode(:,2:end-1),initial.XmidIHS,initial.Anode,initial.Amid,initial.Fnode,initial.Fmid,disc,initial.separator);
    z=[z;extra(:)];
end
research=struct('provider',p,'config',c,'center',[],'design',[],'buildOnly',true,'enforceInsertionDerivatives',anchor);
research.lengthGuard=lengthGuard;
research.additionalObstacleGuard=0;
if isfield(numerics,'additionalObstacleGuard')
    research.additionalObstacleGuard=numerics.additionalObstacleGuard;
end
if isfinite(trustRadius), research.center=z; research.config.trustRadius=trustRadius; end
opts=struct('hessianApproximation',hessian,'maxIter',1500,'printLevel',5);
nlp=buildCasadiTrackableIHSNLP(task.model,solverScene,disc,initial,opts,research);
scale=ones(size(z));
if scaled
    % Finite bound half-ranges: forces only without a trust region, all
    % bounded coordinates when the diagnostic has a finite trust region.
    force=isfinite(nlp.lbz) & isfinite(nlp.ubz);
    scale(force)=(nlp.ubz(force)-nlp.lbz(force))/2;
end
assert(all(isfinite(scale) & scale>0),'ABFD:InvalidScaling','Nonpositive decision range.');
y=MX.sym('scaled_decision',numel(z),1);
[objective,equality,inequality]=nlp.eval(scale.*y);
options=nlp.opts; options.ipopt.mu_strategy=barrier;
if strcmp(barrier,'monotone'), options.ipopt.mu_init=1e-6; end
solver=nlpsol('abfd_diagnostic_solver','ipopt', ...
    struct('x',y,'f',objective,'g',[equality;inequality]),options);
[j,eq,ineq]=nlp.eval(z);
report=struct('taskId',task.taskId,'taskHash',abfdHash(task),'checkpoint',checkpoint,'hessian',hessian, ...
    'checkpointKind',checkpointKind, ...
    'scaledForces',scaled,'barrier',barrier,'enforceInsertionDerivatives',anchor, ...
    'trustRadius',trustRadius,'lengthGuard',lengthGuard, ...
    'collisionSmoothing',solverScene.collision.smoothingEps, ...
    'additionalObstacleGuard',research.additionalObstacleGuard, ...
    'scalingRule','finite bound half-range; unity otherwise','scaledVariableCount',sum(scale~=1), ...
    'initialObjective',full(j),'initialMaxEq',max(abs(full(eq))),'initialMaxIneq',max([0;full(ineq)]));
writeAbfdJson(fullfile(outputDir,'diagnostic_started.json'),report);
timer=tic; sol=solver('x0',z./scale,'lbx',nlp.lbz./scale,'ubx',nlp.ubz./scale,'lbg',nlp.lbg,'ubg',nlp.ubg);
report.solveTime=toc(timer); stats=solver.stats(); z=full(sol.x).*scale;
[j,eq,ineq]=nlp.eval(z); report.objective=full(j); report.maxEq=max(abs(full(eq)));
report.maxIneq=max([0;full(ineq)]); report.status=stats.return_status; report.iterations=stats.iter_count;
[X,Xm,An,Am,Fn,Fm]=unpackIHSDecisionImplicit(z(1:nlp.baseNumZ),task.scene,disc);
traj=struct('t',disc.tNode,'tc',disc.tMid,'Xnode',X,'Xmid',Xm,'Anode',An,'Amid',Am,'Unode',Fn,'Umid',Fm);
report.passed=false; bundle=[];
if stats.success && report.maxEq<=c.constraintTolerance && report.maxIneq<=c.constraintTolerance
    refs=reconstructIhsidContinuousReference(traj,p,c.sampleTime);
    report.dense=validateAbfdReference(refs,task.model,task.scene,c);
    if report.dense.passed
        bundle=buildAbfdReferenceBundle(traj,task.model,task.scene,disc,p,c); report.passed=true;
    end
end
save(fullfile(outputDir,'diagnostic.mat'),'report','traj','z','bundle','task','c');
writeAbfdJson(fullfile(outputDir,'diagnostic.json'),report); disp(report);
end
