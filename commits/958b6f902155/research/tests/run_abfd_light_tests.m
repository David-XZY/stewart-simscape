function report=run_abfd_light_tests()
root=setupAbfdResearch(); addpath(fullfile(root,'controller','tests'));
tests={@testDynamics,@testGeometry,@testDividerPlacement,@testStartPoseSampling,@testReference,@testRiccatiTube,@testRootMomentDerivatives,@testResidual,@testInertialRegression, ...
    @testSplineAndInitialGuess,@testNlpConstruction,@testHsSeedProjection,@testGuardedPlannerContract,@testFrozenMultimodalityAudit,@testSampledPrediction,@testActuatorPrediction,@testCodesignFinalAcceptance,@testTubeMeanMetrics,@testRuntimeControl,@testConfiguration, ...
    @testExperimentAcceptance,@testIntervalObserver,@()testReducedHsNlp([],true), ...
    @()testReducedHsNlp([],false,'local-midpoint'),@()testReducedHsNlp([],true,'local-midpoint'), ...
    @()testCovarianceShooting([],'symmetric'), ...
    @test_03_directory_contract,@test_20_hermite_reference_contract};
results=struct('name',{},'passed',{},'seconds',{},'message',{});
for k=1:numel(tests)
    name=func2str(tests{k}); timer=tic;
    try
        tests{k}(); passed=true; message=''; catch e, passed=false; message=getReport(e,'extended','hyperlinks','off'); end
    results(end+1)=struct('name',name,'passed',passed,'seconds',toc(timer),'message',message); %#ok<AGROW>
    fprintf('%s: %s\n',name,string(passed)); if ~passed, fprintf('%s\n',message); end
end
report=struct('scope','local mathematical/contracts only; no training or full simulation', ...
    'passed',all([results.passed]),'results',results);
writeAbfdJson(fullfile(root,'results','abfd','local_checks.json'),report);
assert(report.passed,'ABFD:LightTestsFailed','One or more local checks failed.');
end
function testDynamics()
m=buildOptModelCustom(); p=buildDynamicsProvider(m); ad=buildCasadiCompositeDynamics(p);
stream=RandStream('mt19937ar','Seed',19);
for k=1:12
    q=m.qHome+.02*randn(stream,6,1); v=.03*randn(stream,6,1); a=.1*randn(stream,6,1);
    F=p.inverse(q,v,a); o=p.evaluate(q,v,a); [W,~]=ad.wrench(q,v,a);
    assert(norm(full(W)-o.W)<1e-7); assert(norm(p.forward([q;v],F)-[v;a])<1e-8);
    [A,B]=ad.linearize([q;v],F); x=[q;v]; numeric=zeros(12);
    for j=1:12, d=zeros(12,1); d(j)=1e-6; numeric(:,j)=(p.forward(x+d,F)-p.forward(x-d,F))/2e-6; end
    assert(norm(full(A)-numeric,'fro')/max(1,norm(numeric,'fro'))<1e-5);
    assert(norm(full(B(7:12,:))-o.H\o.Jv.','fro')<1e-8);
    velocity=[v(1:3);rpyRateToSpatialVelocity(q(4:6),v(4:6))];
    assert(abs(F.'*o.Jv*velocity-(o.Jv.'*F).'*velocity)<1e-8);
end
end
function testGeometry()
m=buildOptModelCustom(); p=buildDynamicsProvider(m); q=m.qHome+[.01;0;.02;.2;-.1;.3];
v=[.02;-.01;.01;.04;-.03;.02]; a=[.1;.02;0;.01;.02;-.03];
[R,E,Ed]=abfdRotationExpressions(q(4:6),v(4:6));
assert(norm(spatialAccelToRpyAccel(q(4:6),v(4:6),E*a(4:6)+Ed*v(4:6))-a(4:6))<1e-12);
assert(norm(rotationErrorSO3(R,R))<1e-10);
assert(abs(norm(rotationErrorSO3(diag([1,-1,-1]),eye(3)))-pi)<1e-8);
ref=struct('q',q,'qd',v,'qdd',a); K=[25*eye(6),10*eye(6)];
for variant={'d0','fixed','tvlqr'}
    [F,~]=computeGeometricInverseDynamicsForce([q;v],ref,K,p,variant{1});
    assert(norm(F-p.inverse(q,v,a))<1e-7);
    ctl=buildGeometricControllerAD(p,variant{1}); x=[q;v]+.001*(1:12).'; rr=[q;v;a];
    jac=full(ctl.jacobian(x,rr,K)); fd=zeros(6,12);
    for j=1:12
        d=zeros(12,1); d(j)=1e-6;
        fp=computeGeometricInverseDynamicsForce(x+d,ref,K,p,variant{1});
        fm=computeGeometricInverseDynamicsForce(x-d,ref,K,p,variant{1}); fd(:,j)=(fp-fm)/2e-6;
    end
    assert(norm(jac-fd,'fro')/max(1,norm(fd,'fro'))<2e-4);
end
end
function testDividerPlacement()
task=sampleTransferTask(20260906,'train'); s=task.scene;
assert(numel(s.hood.obstacles)==4 && s.dividerPlacement.version==2);
o=s.hood.obstacles(end); cylinder=s.objectCylinder;
poses=[s.q0,s.qWaypoint+(s.qGoal-s.qWaypoint)*linspace(0,1,21)];
for k=1:size(poses,2)
    q=poses(:,k); R=rpy2rotmZYX(q(4:6));
    if k==1, n=s.dividerPlacement.startNormal; else, n=s.dividerPlacement.insertionNormal; end
    center=q(1:3)+R*cylinder.center_P; axis=R*cylinder.axis_P;
    support=.5*cylinder.length*abs(axis.'*n)+cylinder.radius*norm(cross(axis,n));
    gap=(o.center_S-center).'*n-o.halfSize.'*abs(o.R_S.'*n)-support;
    assert(gap>=s.collision.safeDistance+.005-1e-12);
end
q=s.dividerPlacement.witnessPose; R=rpy2rotmZYX(q(4:6));
delta=R.'*(o.center_S-q(1:3))-cylinder.center_P;
axial=cylinder.axis_P.'*delta;
assert(abs(axial)<cylinder.length/2 && norm(delta-axial*cylinder.axis_P)<cylinder.radius);
base=buildCylinderBoxTransferScene(task.model);
unchanged=buildMultiObstacleScene(base,false);
assert(isequal(base,unchanged));
end
function testStartPoseSampling()
task=sampleTransferTask(20260907,'train'); s=task.scene; m=task.model;
assert(isfield(s,'initialPoseResampling') && s.initialPoseResampling.attempts>=1);
original=sgpIK(s.initialPoseResampling.originalPose,m);
assert(any(original.L<m.lmin | original.L>m.lmax));
kin=sgpIK(s.q0,m); assert(all(kin.L>=m.lmin & kin.L<=m.lmax));
again=sampleTransferTask(20260907,'train'); assert(isequal(s.q0,again.scene.q0));
distances=evaluateCylinderBoxDistanceNumeric(s.q0,s);
required=s.collision.safeDistance*ones(numel(distances.distances),1);
required(1)=s.collision.stage1ConstraintDistance;
assert(all(distances.distances(:)>=required));
end
function testReference()
m=buildOptModelCustom(); p=buildDynamicsProvider(m); t=[0,.1,.2]; tc=[.05,.15];
q=@(t)m.qHome+ones(6,1)*(.01*t.^5); v=@(t)ones(6,1)*(.05*t.^4); a=@(t)ones(6,1)*(.2*t.^3);
traj=struct('t',t,'tc',tc,'Xnode',[q(t);v(t)],'Xmid',[q(tc);v(tc)],'Anode',a(t),'Amid',a(tc));
r=reconstructIhsidContinuousReference(traj,p,.013);
assert(max(abs(r.q-q(r.t)),[],'all')<1e-11);
assert(max(abs(r.qd-v(r.t)),[],'all')<1e-10);
assert(max(abs(r.qdd-a(r.t)),[],'all')<1e-8);
assert(r.maxDynamicsResidual<1e-8);
failed=false; try
        reconstructIhsidContinuousReference(rmfield(traj,'Xmid'),p,.01); catch e, failed=strcmp(e.identifier,'ABFD:MissingHsData'); end
assert(failed);
end
function testRiccatiTube()
dt=.01; A=repmat([eye(6),dt*eye(6);zeros(6),eye(6)],1,1,10);
B=repmat([dt^2/2*eye(6);dt*eye(6)],1,1,10);
s=buildFiniteHorizonFullStateLqr(A,B,eye(12),eye(6),10*eye(12)); assert(all(isfinite(s.K),'all'));
G=zeros(12,6,10); tube=propagateTrackingTube(repmat(eye(12),1,1,10),G,zeros(6),eye(12),zeros(12));
assert(norm(tube.P(:,:,end)-eye(12))<1e-12);
m=buildOptModelCustom(); p=buildDynamicsProvider(m); q=repmat(m.qHome,1,4);
F=repmat(p.inverse(m.qHome,zeros(6,1),zeros(6,1)),1,4);
refs=struct('q',q,'qd',zeros(6,4),'qdd',zeros(6,4),'Fleg',F,'t',[0,.01,.02,.03]);
c=makeAbfdConfig(struct('controllerMode','geometric-D'));
metric=run_A0_trackability_metric(refs,p,c); assert(isfinite(metric.predictedCost));
end
function testResidual()
m=buildOptModelCustom(); net=fixture(m); p=buildDynamicsProvider(m,'residual-B',net,true); ad=buildCasadiCompositeDynamics(p);
q=m.qHome; v=.01*ones(6,1); a=.02*ones(6,1); o=p.evaluate(q,v,a); [W,S]=ad.wrench(q,v,a);
assert(norm(full(W)-o.W)<1e-8 && norm(full(S)-o.Sigma)<1e-8);
assert(all(eig(o.Sigma)>0)); failed=false;
try
        buildDynamicsProvider(m,'residual-B',net); catch e, failed=strcmp(e.identifier,'ABFD:UntrainedAsset'); end
assert(failed); p0=buildDynamicsProvider(m);
session=struct('t',[0,.01,.02],'forceTime',[0,.01,.02], ...
    'q',repmat(q,1,3),'qd',zeros(6,3),'qdd',zeros(6,3), ...
    'actualForce',repmat(p0.inverse(q,zeros(6,1),zeros(6,1)),1,3), ...
    'taskId','fixture','loadId','fixture','trajectoryId','fixture','forceSource','measured-actuator-output');
d=buildResidualWrenchTarget(session,m); assert(max(abs(d.residual),[],'all')<1e-8);
session.forceSource='command'; failed=false;
try
        buildResidualWrenchTarget(session,m); catch e, failed=strcmp(e.identifier,'ABFD:ActualForceRequired'); end
assert(failed);
end
function testSplineAndInitialGuess()
task=sampleTransferTask(19,'train'); m=task.model; s=task.scene; p=buildDynamicsProvider(m);
cp=zeros(6,10); cp(:,1:3)=repmat(s.q0,1,3); cp(:,8:10)=repmat(s.qWaypoint,1,3);
for j=4:7, cp(:,j)=s.q0+(j-3)/5*(s.qWaypoint-s.q0); end
[q,v,a]=evaluateTrajectorySpline(cp,[0,s.phase.durationApproach],s.phase.durationApproach);
assert(norm(q(:,1)-s.q0)<1e-10 && norm(q(:,2)-s.qWaypoint)<1e-10);
assert(norm(v,'fro')<1e-8 && norm(a,'fro')<1e-7);
t=.37*s.phase.durationApproach; h=1e-5;
[~,v,a]=evaluateTrajectorySpline(cp,t,s.phase.durationApproach);
[qp,vp]=evaluateTrajectorySpline(cp,t+h,s.phase.durationApproach);
[qm,vm]=evaluateTrajectorySpline(cp,t-h,s.phase.durationApproach);
assert(norm((qp-qm)/2/h-v)<1e-6 && norm((vp-vm)/2/h-a)<1e-5);
disc=buildTwoPhaseIHSDiscretization(s,4,2);
[z,initial]=buildLearnedInitialGuessIHSID(cp,m,s,disc,p); assert(all(isfinite(z)));
assert(norm(initial.Xnode(:,end)-[s.qGoal;s.qdGoal])<1e-8);
[legacy,~]=buildInitialGuessTwoPhaseIHSImplicit(m,s,disc); assert(numel(legacy)==numel(z));
end
function testNlpConstruction()
m=buildOptModelCustom(); s=buildCylinderBoxTransferScene(m); d=buildTwoPhaseIHSDiscretization(s,2,1);
[z,initial]=buildInitialGuessTwoPhaseIHSImplicit(m,s,d); p=buildDynamicsProvider(m);
c=makeAbfdConfig(struct('solverMaxCpuTime',60));
research=struct('provider',p,'config',c,'center',[],'design',[],'buildOnly',true);
nlp=buildCasadiTrackableIHSNLP(m,s,d,initial,struct(),research); [value,eq,ineq]=nlp.eval(z);
assert(isfinite(full(value)) && all(isfinite(full(eq))) && all(isfinite(full(ineq))));
assert(nlp.opts.ipopt.max_cpu_time==60 && nlp.opts.ipopt.max_iter==300);
raw=struct('t',d.tNode,'tc',d.tMid,'Xnode',initial.Xnode,'Xmid',initial.XmidIHS,'Anode',initial.Anode,'Amid',initial.Amid);
refs=reconstructIhsidContinuousReference(raw,p,d.h/2);
c.controllerMode='geometric-D'; c.designMode='codesign-A';
design=buildAbfdSchedule(refs,p,c); design.nominal=refs;
research.config=c; research.design=design;
nlpA=buildCasadiTrackableIHSNLP(m,s,d,initial,struct(),research);
[value,~,ineq]=nlpA.eval(z); assert(isfinite(full(value)) && all(isfinite(full(ineq))));
assert(nlpA.sizes.numIneq>nlp.sizes.numIneq);
end
function testConfiguration()
c=makeAbfdConfig(); assert(c.candidateCount==8 && c.refineCount==2);
assert(c.solverMaxIterations==300 && isinf(c.solverMaxCpuTime));
budget=makeAbfdConfig(struct('solverMaxIterations',2000,'solverMaxCpuTime',600));
assert(~strcmp(abfdHash(c),abfdHash(budget)));
rejected=false;
try
    makeAbfdConfig(struct('solverMaxIterations',0));
catch
    rejected=true;
end
assert(rejected,'Invalid solve budgets must be rejected.');
assert(~c.qpEnabled && ~c.governorEnabled); assert(strcmp(abfdHash(c),abfdHash(c)));
matrix=buildAbfdExperimentMatrix(c); assert(numel(matrix)==13);
assert(strcmp(matrix(2).config.controllerMode,'fullstate-tvlqr'));
assert(numel(buildAbfdCaseMatrix())==12);
end
function testRuntimeControl()
m=buildOptModelCustom(); p=buildDynamicsProvider(m); s=buildCylinderBoxTransferScene(m);
q=repmat(m.qHome,1,3); z=zeros(6,3); F=repmat(p.inverse(m.qHome,zeros(6,1),zeros(6,1)),1,3);
b=struct('publishPassed',true,'providerHash',p.hash,'model',m,'scene',s, ...
    'refs',struct('q',q,'qd',z,'qdd',z,'Fleg',F,'t',[0,.01,.02]));
state=struct('integral',zeros(6,1),'previousVelocity',zeros(6,1),'appliedForce',F(:,1), ...
    'previousForce',F(:,1),'wrenchEstimate',zeros(6,1),'compensation',zeros(6,1), ...
    'governorPosition',zeros(3,1),'governorRate',zeros(3,1));
for mode={'geometric-D','fullstate-tvlqr','baseline-lqi'}
    c=makeAbfdConfig(struct('controllerMode',mode{1})); r=makeAbfdRuntime(b,p,c);
    [out,~]=stepAbfdController(0,[m.qHome;zeros(6,1)],state,r); assert(norm(out-F(:,1))<1e-7);
    if strcmp(mode{1},'baseline-lqi')
        ad=buildCasadiCompositeDynamics(p); [A,B]=ad.linearize([m.qHome;zeros(6,1)],F(:,1));
        Z=expm([full(A),full(B);zeros(6,18)]*c.sampleTime);
        K=r.lqi.gains(:,:,1); C=[eye(6),zeros(6)];
        closed=[Z(1:12,1:12)-Z(1:12,13:18)*K(:,1:12),-Z(1:12,13:18)*K(:,13:18); ...
            c.sampleTime*C,(1-c.sampleTime*r.lqi.leak)*eye(6)];
        assert(max(abs(eig(closed)))<1);
    end
    c.dobEnabled=true; c.governorEnabled=true; c.antiWindupEnabled=strcmp(mode{1},'baseline-lqi');
    r=makeAbfdRuntime(b,p,c); [out,~]=stepAbfdController(0,[m.qHome;zeros(6,1)],state,r);
    assert(norm(out-F(:,1))<1e-7);
end
[corrected,~,P]=stepAbfdExternalObserver([m.qHome;zeros(6,1)],eye(12)*1e-4,m.qHome,F(:,1),p,.01);
assert(norm(corrected-[m.qHome;zeros(6,1)])<1e-8 && min(eig(P))>0);
end
function n=fixture(m)
n=struct('schemaVersion',1,'kind','residual-wrench-tanh','status','test-fixture', ...
    'physical',struct('mass',m.dynamics.totalMass,'com',m.dynamics.comP,'inertia',m.dynamics.inertiaAtCOM_P), ...
    'inputMean',zeros(12,1),'inputScale',ones(12,1),'outputScale',ones(6,1),'varianceCalibration',1, ...
    'layers',[struct('W',reshape(sin(1:192),16,12)*.02,'b',zeros(16,1)), ...
    struct('W',reshape(cos(1:192),12,16)*.02,'b',[ones(6,1);-4*ones(6,1)])]);
end
