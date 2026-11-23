function report=testConditionalActuatorPropagation(asset,temporalFile,legacyFile,bundle,calibrationFile,plain,startupInertia)
% Moving-reference, full D Jacobian and joint actuator/VAR covariance contract.
import casadi.*
if nargin<6, plain=makeHighFidelityPlantProfile('high-fidelity'); end
if nargin<7, startupInertia=false; end
c=makeAbfdConfig(struct('modelMode','residual-B','residualFile',asset,'staticBalanceFile',calibrationFile, ...
    'predictionProfile','actuator-var-v3','temporalFile',temporalFile,'allowPilotTemporal',true, ...
    'plantProfile','high-fidelity','plantOverrides',plain,'controllerMode','geometric-D','designMode','codesign-A'));
measurement=[];
if startupInertia
    c.inertiaCalibrationFile=calibrationFile;
    measurement=readMeasuredInertiaRegression(calibrationFile,bundle,plain);
end
base=buildDynamicsProvider(bundle.model,'residual-B',asset,false,bundle.refs.q(:,1)+plain.initialPoseOffset,measurement);
[p,c,profile]=configureAbfdStaticBalance(base,c,plain,bundle.model,bundle.refs.q(:,1));
temporal=importResidualTemporalModel(temporalFile,asset,true,p);
validateAbfdTemporalConditioning(p,temporal,profile);
assert(~p.biasCalibration.uncertaintyValidated,'ABFD:ConditionalAcceptance','Pilot import must not claim coverage acceptance.');
rejections=struct();
rejections.unconditionedMean=rejects(@()importResidualTemporalModel(temporalFile,asset,true,base),'ABFD:TemporalConditioningMismatch');
if startupInertia
    unconditioned=rmfield(temporal,'conditioning');
    rejections.unconditionedNoise=rejects(@()validateAbfdTemporalConditioning(p,unconditioned),'ABFD:UncalibratedConditionalUncertainty');
    stale=temporal; stale.conditioning.kind='independent-initial-static-balance-v1';
    rejections.oldStaticProtocol=rejects(@()validateAbfdTemporalConditioning(p,stale),'ABFD:TemporalStartupMismatch');
else
    rejections.unconditionedNoise=rejects(@()importResidualTemporalModel(legacyFile,asset,true,p),'ABFD:UncalibratedConditionalUncertainty');
end
rejections.pilotPermission=rejects(@()importResidualTemporalModel(temporalFile,asset,false,p),'ABFD:TemporalPilot');
bad=profile; bad.initialAppliedForce(1)=bad.initialAppliedForce(1)+1;
rejections.preload=rejects(@()validateAbfdTemporalConditioning(p,temporal,bad),'ABFD:TemporalConditioningPreload');
bad=profile; bad.payloadMassScale=bad.payloadMassScale+.01;
rejections.plant=rejects(@()validateAbfdTemporalConditioning(p,temporal,bad),'ABFD:TemporalConditioningPlant');
bad=p; bad.biasCalibration.wrenchOffset(1)=bad.biasCalibration.wrenchOffset(1)+1;
rejections.offset=rejects(@()validateAbfdTemporalConditioning(bad,temporal,profile),'ABFD:TemporalConditioningBalance');
bad=temporal; bad.staticNetworkHash=repmat('f',1,64);
rejections.network=rejects(@()validateAbfdTemporalConditioning(p,bad,profile),'ABFD:TemporalConditioningMismatch');
assert(all(structfun(@(v)v,rejections)),'ABFD:ConditionalGuard','A mismatched context was accepted.');
t=[0,.015,.03]; X=repmat([bundle.refs.q(:,1);zeros(6,1)],1,3); A=zeros(6,3);
X(1,2:3)=X(1,2:3)+[2e-5,5e-5]; X(5,2:3)=X(5,2:3)+[1e-5,2e-5];
X(7,2:3)=[.001,.002]; X(11,2:3)=[.0005,.001];
raw=struct('t',t([1,3]),'tc',t(2),'Xnode',X(:,[1,3]),'Xmid',X(:,2),'Anode',A(:,[1,3]),'Amid',A(:,2));
refs=reconstructIhsidContinuousReference(raw,p,c.sampleTime); design=buildAbfdSchedule(refs,p,c);
runtime=struct('bundle',struct('refs',refs,'model',bundle.model),'provider',p,'config',c,'design',design,'initialState',X(:,1));
options=struct('queryTime',t,'maxStep',.001,'temporalFile',temporalFile,'allowPilotTemporal',true);
numeric=predictAbfdClosedLoopMean(runtime,p,profile,options);
decision=MX.sym('conditional_reference',54); knots=reshape(decision,18,3); initial=[X;A]; initial=initial(:);
timer=tic;
rollout=buildAbfdActuatorRolloutAD(knots(1:12,:),knots(13:18,:),t,p,design,c,profile,temporal,.001);
[cost,constraints]=buildAbfdTrackingExpressions(knots(1:12,:),knots(13:18,:),[],t,p,design,c,struct('hood',struct('obstacles',[])));
n=size(rollout.P{end},1);
outputs=[rollout.mean(:);reshape(rollout.P{end},n*n,1);cost;constraints];
evaluation=Function('conditional_rollout_evaluation',{decision},{outputs}); values=full(evaluation(initial));
states=reshape(values(1:72),24,3); covariance=reshape(values(73:72+n*n),n,n);
meanError=norm(states(1:12,:)-numeric.x,'fro');
covarianceError=norm(covariance-numeric.finalJointCovariance,'fro')/norm(numeric.finalJointCovariance,'fro');
assert(meanError<1e-8 && covarianceError<1e-8,'ABFD:ConditionalRollout','Numeric and symbolic conditional propagation differ.');
direction=sin((1:54).'); direction=direction/norm(direction);
derivative=Function('conditional_rollout_direction',{decision},{jtimes(outputs,decision,direction)});
analytic=full(derivative(initial)); steps=[1e-6,1e-7,1e-8,1e-9,1e-10]; errors=zeros(size(steps));
parts=zeros(numel(steps),3);
for k=1:numel(steps)
    h=steps(k); finite=(full(evaluation(initial+h*direction))-full(evaluation(initial-h*direction)))/(2*h);
    errors(k)=norm(analytic-finite)/max(1,norm(finite));
    ranges={1:72,73:72+n*n,73+n*n:numel(analytic)};
    for j=1:3
        ids=ranges{j}; parts(k,j)=norm(analytic(ids)-finite(ids))/max(1,norm(finite(ids)));
    end
    fprintf('conditional derivative h=%g mean=%g jointP=%g constraints=%g total=%g\n',h,parts(k,:),errors(k));
end
assert(all(isfinite(analytic)) && any(all(parts<1e-4,2) & errors(:)<1e-4), ...
    'ABFD:ConditionalDerivative','Full D/actuator/VAR derivative mismatch across the finite-difference convergence range.');
report=struct('passed',true,'meanError',meanError,'jointCovarianceRelativeError',covarianceError, ...
    'jointStateCount',n,'steps',steps,'derivativeErrors',errors,'seconds',toc(timer), ...
    'derivativeParts',parts, ...
    'rejections',rejections,'providerHash',p.hash,'temporalHash',temporal.assetHash, ...
    'scope','Moving short reference; full D, actuator and conditional VAR propagation; no full-horizon optimization or coverage acceptance');
end
function yes=rejects(action,identifier)
yes=false;
try
    action();
catch e
    yes=strcmp(e.identifier,identifier);
end
end
