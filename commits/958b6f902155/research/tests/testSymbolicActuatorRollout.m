function report=testSymbolicActuatorRollout(asset,temporalFile,controller)
% Short mathematical contract using explicitly supplied pilot assets.
import casadi.*
temporal=importResidualTemporalModel(temporalFile,asset,true);
m=buildOptModelCustom(); p=buildDynamicsProvider(m,'residual-B',asset); profile=makeHighFidelityPlantProfile('high-fidelity');
c=makeAbfdConfig(struct('predictionProfile','sampled-hold-v2','controllerMode',controller, ...
    'modelMode','residual-B','residualFile',asset,'plantProfile','high-fidelity'));
t=[0,.015,.03]; X=repmat([m.qHome;zeros(6,1)],1,3); A=zeros(6,3);
hs=struct('t',[0,.03],'tc',.015,'Xnode',X(:,[1,3]),'Xmid',X(:,2),'Anode',A(:,[1,3]),'Amid',A(:,2));
refs=reconstructIhsidContinuousReference(hs,p,c.sampleTime); design=buildAbfdSchedule(refs,p,c);
runtime=struct('bundle',struct('refs',refs,'model',m),'provider',p,'config',c,'design',design,'initialState',X(:,1));
options=struct('queryTime',t,'maxStep',.001,'temporalFile',temporalFile,'allowPilotTemporal',true);
numeric=predictAbfdClosedLoopMean(runtime,p,profile,options);
decision=MX.sym('reference_knots',54); knots=reshape(decision,18,3);
timer=tic; rollout=buildAbfdActuatorRolloutAD(knots(1:12,:),knots(13:18,:),t,p,design,c,profile,temporal,.001);
outputs=[rollout.mean(:);reshape(rollout.P{end}(1:12,1:12),144,1)];
evaluation=Function('full_symbolic_rollout',{decision},{outputs}); initial=[X;A]; initial=initial(:);
values=full(evaluation(initial)); states=reshape(values(1:72),24,3);
meanError=norm(states(1:12,:)-numeric.x,'fro');
covarianceError=norm(reshape(values(73:end),12,12)-numeric.P(:,:,end),'fro')/norm(numeric.P(:,:,end),'fro');
assert(meanError<1e-8 && covarianceError<1e-8,'ABFD:SymbolicRollout','Symbolic rollout differs from numerical prediction.');
stream=RandStream('mt19937ar','Seed',31); direction=randn(stream,54,1); direction=direction/norm(direction);
derivative=Function('full_rollout_direction',{decision},{jtimes(outputs,decision,direction)});
analytic=full(derivative(initial)); steps=[1e-6,1e-7,1e-8]; errors=zeros(size(steps));
for k=1:numel(steps)
    h=steps(k); finite=(full(evaluation(initial+h*direction))-full(evaluation(initial-h*direction)))/(2*h);
    errors(k)=norm(analytic-finite)/max(1,norm(finite));
end
assert(errors(end)<1e-4,'ABFD:RolloutDerivative','Candidate-dependent rollout derivative mismatch.');
hermite=evaluateAbfdHermiteExpressions(X,A,t,refs.t);
hermiteError=norm(hermite-[refs.q;refs.qd;refs.qdd],'fro'); assert(hermiteError<1e-8);
report=struct('passed',true,'controller',controller,'meanError',meanError, ...
    'covarianceRelativeError',covarianceError,'derivativeSteps',steps,'derivativeErrors',errors, ...
    'hermiteError',hermiteError,'seconds',toc(timer), ...
    'scope','short stationary reference, real pilot weights; kernel mathematics only, no full NLP or performance acceptance');
end
