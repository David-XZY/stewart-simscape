function report=testActuatorMomentShooting(asset,temporalFile,bundle,calibrationFile,plain)
% Real anchored weights, full VAR order, off-clock moving HS knots and D.
% Short mathematical equivalence test, not a feasible trajectory or experiment.
import casadi.*
timer=tic;
c=makeAbfdConfig(struct('modelMode','residual-B','residualFile',asset, ...
    'staticBalanceFile',calibrationFile,'inertiaCalibrationFile',calibrationFile, ...
    'predictionProfile','actuator-var-v3','temporalFile',temporalFile,'allowPilotTemporal',true, ...
    'plantProfile','high-fidelity','plantOverrides',plain,'controllerMode','geometric-D', ...
    'designMode','codesign-A','plannerProfile','guarded-exact-v1', ...
    'solverFormulation','covariance-shooting','covarianceShootingInterval',.01));
[p,c,profile]=buildAbfdProviderForTask(bundle.model,c,plain,bundle.refs.q(:,1));
temporal=importResidualTemporalModel(temporalFile,asset,true,p);
t=[0,.015,.03]; X=repmat([bundle.refs.q(:,1);zeros(6,1)],1,3); A=zeros(6,3);
X(1,2:3)=X(1,2:3)+[2e-5,5e-5]; X(5,2:3)=X(5,2:3)+[1e-5,2e-5];
X(7,2:3)=[.001,.002]; X(11,2:3)=[.0005,.001];
raw=struct('t',t([1,3]),'tc',t(2),'Xnode',X(:,[1,3]),'Xmid',X(:,2),'Anode',A(:,[1,3]),'Amid',A(:,2));
refs=reconstructIhsidContinuousReference(raw,p,c.sampleTime); design=buildAbfdSchedule(refs,p,c);
runtime=struct('bundle',struct('refs',refs,'model',bundle.model),'provider',p,'config',c, ...
    'design',design,'initialState',[X(1:6,1)+profile.initialPoseOffset;zeros(6,1)]);
numeric=predictAbfdClosedLoopMean(runtime,p,profile, ...
    struct('queryTime',t,'maxStep',.001,'temporalFile',temporalFile,'allowPilotTemporal',true));
decision=MX.sym('moment_reference',54); knots=reshape(decision,18,3); initial=[X;A]; initial=initial(:);
scene=struct('hood',struct('obstacles',[]));
ordinary=buildAbfdActuatorRolloutAD(knots(1:12,:),knots(13:18,:),t,p,design,c,profile,temporal,.001);
[baseCost,baseConstraints]=buildAbfdTrackingExpressions(knots(1:12,:),knots(13:18,:),[],t,p,design,c,scene);
active=c; active.covarianceShootingActive=true;
lifted=buildAbfdActuatorRolloutAD(knots(1:12,:),knots(13:18,:),t,p,design,active,profile,temporal,.001);
[liftCost,liftConstraints,diag]=buildAbfdTrackingExpressions(knots(1:12,:),knots(13:18,:),[],t,p,design,active,scene);
% The cost builder creates its own equivalent auxiliary symbols. Substitute
% them explicitly; accidental hidden free variables must not pass Function.
mapped=substitute([liftCost;liftConstraints],diag.lifting.variables,lifted.lifting.variables);
baseP=horzcat(ordinary.P{:}); liftP=horzcat(lifted.P{:});
baseOutput=[ordinary.mean(:);ordinary.command(:);baseP(:);baseCost;baseConstraints];
liftOutput=[lifted.mean(:);lifted.command(:);liftP(:);mapped];
nlp=struct('z',[decision;lifted.lifting.variables],'physicalNumZ',54,'tube',struct('lifting',lifted.lifting));
baseFunction=Function('moment_uncut_value',{decision},{baseOutput});
liftFunction=Function('moment_cut_value',{nlp.z},{liftOutput});
[seed,seedReport]=buildAbfdCovarianceShootingSeed(nlp,initial);
expected=full(baseFunction(initial)); actual=full(liftFunction(seed)); n=size(ordinary.P{1},1);
assert(n==36+size(temporal.transition,1),'ABFD:MomentVarOrder','The full VAR order must be retained.');
ranges={1:90,91:90+3*n*n,91+3*n*n:numel(expected)};
valueErrors=partErrors(expected,actual,ranges);
meanError=norm(reshape(actual(1:72),24,3)-reshape(expected(1:72),24,3),'fro');
state=reshape(actual(1:72),24,3); numericMeanError=norm(state(1:12,:)-numeric.x,'fro');
finalP=reshape(actual(91+2*n*n:90+3*n*n),n,n);
numericPError=norm(finalP-numeric.finalJointCovariance,'fro')/max(1,norm(numeric.finalJointCovariance,'fro'));
assert(norm(finalP(1:30,37:end),'fro')>1e-8,'ABFD:MomentCrossCovariance','The fixture must exercise nonzero cross moments.');
for k=1:2
    point=initial+1e-9*sin((1:54).'+k);
    z=buildAbfdCovarianceShootingSeed(nlp,point);
    valueErrors=max(valueErrors,partErrors(full(baseFunction(point)),full(liftFunction(z)),ranges));
end
direction=sin((1:54).'); direction=direction/norm(direction);
v=MX.sym('moment_tangent',numel(seed)); tangent=[direction;zeros(numel(seed)-54,1)];
for k=1:numel(lifted.lifting.cutTimes)
    derivative=Function('moment_cut_tangent',{nlp.z,v}, ...
        {jtimes(lifted.lifting.predictions{k},nlp.z,v)});
    ids=54+(k-1)*lifted.lifting.factorSize+(1:lifted.lifting.factorSize);
    tangent(ids)=full(derivative(seed,tangent));
end
continuity=Function('moment_continuity_tangent',{nlp.z,v}, ...
    {jtimes(lifted.lifting.equalities,nlp.z,v)});
tangentResidual=norm(full(continuity(seed,tangent)),inf);
baseDerivative=Function('moment_uncut_tangent',{decision},{jtimes(baseOutput,decision,direction)});
liftDerivative=Function('moment_lifted_tangent',{nlp.z,v},{jtimes(liftOutput,nlp.z,v)});
expectedDerivative=full(baseDerivative(initial)); actualDerivative=full(liftDerivative(seed,tangent));
tangentErrors=partErrors(expectedDerivative,actualDerivative,ranges);
steps=[1e-7,1e-8,1e-9]; fdErrors=zeros(numel(steps),3);
for k=1:numel(steps)
    h=steps(k); plus=buildAbfdCovarianceShootingSeed(nlp,initial+h*direction);
    minus=buildAbfdCovarianceShootingSeed(nlp,initial-h*direction);
    finite=full(liftFunction(plus)-liftFunction(minus))/(2*h);
    fdErrors(k,:)=partErrors(finite,actualDerivative,ranges);
end
check=Function('moment_continuity',{nlp.z},{lifted.lifting.equalities});
corrupted=seed; corrupted(55)=corrupted(55)+.01;
corruptionResidual=norm(full(check(corrupted)),inf);
assert(all(valueErrors<1e-8) && numericMeanError<1e-8 && numericPError<1e-8 && ...
    all(tangentErrors<1e-8) && tangentResidual<1e-7 && ...
    any(all(fdErrors<1e-4,2)) && corruptionResidual>=.009, ...
    'ABFD:ActuatorMomentContract','Lifted means, cross moments, constraints or implicit derivatives differ.');
% An ordinary numeric call with the public shooting config has no free
% auxiliary variables; acceptance retains full propagation from its prior.
metric=run_A0_trackability_metric(refs,p,c,design);
assert(isfinite(metric.predictedCost),'ABFD:UnliftedAcceptance','A0 must remain evaluable without auxiliary values.');
report=struct('passed',true,'seconds',toc(timer),'jointStateCount',n, ...
    'residualCount',size(temporal.transition,1),'cuts',lifted.lifting.cutTimes, ...
    'variablesPerCut',lifted.lifting.factorSize,'seed',seedReport, ...
    'meanError',meanError,'numericMeanError',numericMeanError,'numericJointPRelativeError',numericPError, ...
    'valueErrors',valueErrors,'implicitTangentErrors',tangentErrors,'tangentContinuityResidual',tangentResidual, ...
    'finiteDifferenceSteps',steps,'finiteDifferenceErrors',fdErrors,'corruptionResidual',corruptionResidual, ...
    'providerHash',p.hash,'temporalHash',temporal.assetHash, ...
    'scope','Short moving-reference equivalence and derivatives with full D/actuator/VAR; no optimizer or performance acceptance');
end
function errors=partErrors(expected,actual,ranges)
errors=zeros(1,numel(ranges));
for k=1:numel(ranges)
    ids=ranges{k}; errors(k)=norm(expected(ids)-actual(ids))/max(1,norm(expected(ids)));
end
end
