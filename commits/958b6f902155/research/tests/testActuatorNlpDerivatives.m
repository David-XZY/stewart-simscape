function report=testActuatorNlpDerivatives(asset,temporalFile,plantOverrides)
% Explicit real assets; this checks the full NLP composition, not feasibility.
if nargin<3, plantOverrides=struct(); end
import casadi.*
m=buildOptModelCustom(); s=buildCylinderBoxTransferScene(m);
s.q0=m.qHome; s.qWaypoint=m.qHome; s.qGoal=m.qHome;
s.qd0=zeros(6,1); s.qdGoal=zeros(6,1);
s.phase.durationApproach=.03; s.phase.durationInsertion=.015;
d=buildTwoPhaseIHSDiscretization(s,2,1);
[z,initial]=buildInitialGuessTwoPhaseIHSImplicit(m,s,d);
p=buildDynamicsProvider(m,'residual-B',asset);
c=makeAbfdConfig(struct('predictionProfile','actuator-var-v3','controllerMode','geometric-D', ...
    'modelMode','residual-B','residualFile',asset,'temporalFile',temporalFile, ...
    'allowPilotTemporal',true,'plantProfile','high-fidelity','plantOverrides',plantOverrides,'designMode','codesign-A'));
raw=struct('t',d.tNode,'tc',d.tMid,'Xnode',initial.Xnode,'Xmid',initial.XmidIHS,'Anode',initial.Anode,'Amid',initial.Amid);
refs=reconstructIhsidContinuousReference(raw,p,c.sampleTime);
design=buildAbfdSchedule(refs,p,c); design.nominal=refs;
research=struct('provider',p,'config',c,'design',design,'center',[],'buildOnly',true);
timer=tic; nlp=buildCasadiTrackableIHSNLP(m,s,d,initial,struct(),research); buildSeconds=toc(timer);
[cost,eq,inequality]=nlp.eval(z); assert(all(isfinite(full([cost;eq;inequality]))));
symbol=MX.sym('actuator_decision',numel(z)); [J,~,ineq]=nlp.eval(symbol);
stream=RandStream('mt19937ar','Seed',71); direction=randn(stream,numel(z),1); direction=direction/norm(direction);
timer=tic; derivative=Function('actuator_nlp_directional',{symbol},{jtimes([J;ineq],symbol,direction)});
analytic=full(derivative(z)); derivativeSeconds=toc(timer);
steps=[1e-5,1e-6,1e-7,1e-8]; errors=zeros(size(steps));
for k=1:numel(steps)
    h=steps(k); [jp,~,cp]=nlp.eval(z+h*direction); [jm,~,cm]=nlp.eval(z-h*direction);
    finite=full([jp-jm;cp-cm])/(2*h); errors(k)=norm(analytic-finite)/max(1,norm(finite));
    fprintf('actuator NLP h=%g relative=%g nonfinite=%d\n',h,errors(k),sum(~isfinite(analytic)));
end
assert(all(isfinite(analytic)) && errors(end)<1e-4,'ABFD:ActuatorNlpDerivative','Full actuator NLP derivative mismatch.');
sequential=research; sequential.config.designMode='sequential';
base=buildCasadiTrackableIHSNLP(m,s,d,initial,struct(),sequential);
added=nlp.sizes.numIneq-base.sizes.numIneq;
assert(added==(73+numel(s.hood.obstacles))*(d.numNodes+d.numMidpoints),'ABFD:ActuatorConstraintLayout');
metric=run_A0_trackability_metric(refs,p,c,design);
options=struct('queryTime',refs.t,'maxStep',.001,'temporalFile',temporalFile,'allowPilotTemporal',true);
runtime=struct('bundle',struct('refs',refs,'model',m),'provider',p,'config',c,'design',design,'initialState',[refs.q(:,1);refs.qd(:,1)]);
numeric=predictAbfdClosedLoopMean(runtime,p,makeHighFidelityPlantProfile('high-fidelity',plantOverrides),options);
meanError=norm(metric.mean-numeric.x,'fro'); P=cat(3,metric.P{:});
covarianceError=norm(P(:)-numeric.P(:))/norm(numeric.P(:));
assert(meanError<1e-8 && covarianceError<1e-8,'ABFD:ActuatorMetricMismatch','A0 differs from numerical prediction.');
if isfield(plantOverrides,'initialAppliedForce')
    assert(norm(numeric.actualForce(:,1)-plantOverrides.initialAppliedForce)<1e-10, ...
        'ABFD:PredictionPreload','Numerical prediction ignored the explicit physical preload.');
end
bad=numeric.x; bad(5,1)=pi/2; rejected=false;
try
    validateAbfdPredictionDomain(bad,p);
catch e
    rejected=strcmp(e.identifier,'ABFD:PredictionInvalid');
end
assert(rejected,'ABFD:PredictionDomainTest','Singular predicted mean was accepted.');
report=struct('passed',true,'buildSeconds',buildSeconds,'derivativeSeconds',derivativeSeconds, ...
    'steps',steps,'errors',errors,'meanError',meanError,'covarianceRelativeError',covarianceError, ...
    'decisionCount',numel(z),'addedConstraints',added, ...
    'scope','Full candidate-dependent geometric D, actuator/VAR constraints and A0 on a short mathematical fixture; full-horizon optimization and performance remain separate');
end
