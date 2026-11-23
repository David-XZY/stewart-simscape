function report=testCovarianceShooting(asset,coordinates)
% Check equality to the full propagation, including its implicit tangent,
% across off-clock HS knots. No fixture is a feasible training trajectory.
import casadi.*
if nargin<2, coordinates='cholesky'; end
coordinates=validatestring(coordinates,{'cholesky','symmetric'});
m=buildOptModelCustom(); s=buildCylinderBoxTransferScene(m);
s.q0=m.qHome; s.qWaypoint=m.qHome; s.qGoal=m.qHome; s.qd0=zeros(6,1); s.qdGoal=zeros(6,1);
s.phase.durationApproach=.03; s.phase.durationInsertion=.015;
d=buildTwoPhaseIHSDiscretization(s,2,1); [~,initial]=buildInitialGuessTwoPhaseIHSImplicit(m,s,d);
p=buildDynamicsProvider(m); mode='analytic';
if nargin>0 && ~isempty(asset), p=buildDynamicsProvider(m,'residual-B',asset); mode='residual-B'; end
initial.Xnode=repmat([m.qHome;zeros(6,1)],1,d.numNodes);
initial.XmidIHS=repmat([m.qHome;zeros(6,1)],1,d.numMidpoints);
initial.Anode=zeros(6,d.numNodes); initial.Amid=zeros(6,d.numMidpoints);
force=p.inverse(m.qHome,zeros(6,1),zeros(6,1));
initial.Fnode=repmat(force,1,d.numNodes); initial.Fmid=repmat(force,1,d.numMidpoints);
physical=packIHSDecisionImplicit(initial.Xnode(:,2:end-1),initial.XmidIHS,initial.Anode,initial.Amid, ...
    initial.Fnode,initial.Fmid,d,initial.separator);
c=makeAbfdConfig(struct('predictionProfile','sampled-hold-v2','controllerMode','geometric-D', ...
    'dVariant','fixed','designMode','codesign-A','modelMode',mode,'plannerProfile','guarded-exact-v1'));
raw=struct('t',d.tNode,'tc',d.tMid,'Xnode',initial.Xnode,'Xmid',initial.XmidIHS,'Anode',initial.Anode,'Amid',initial.Amid);
refs=reconstructIhsidContinuousReference(raw,p,c.sampleTime); design=buildAbfdSchedule(refs,p,c); design.nominal=refs;
r=struct('provider',p,'config',c,'design',design,'center',physical,'buildOnly',true);
ordinary=buildCasadiTrackableIHSNLP(m,s,d,initial,struct(),r);
r.config.solverFormulation='covariance-shooting'; r.config.covarianceShootingInterval=.02;
if strcmp(coordinates,'symmetric'), r.config.covarianceCoordinates=coordinates; end
r.config=makeAbfdConfig(r.config);
lifted=buildCasadiTrackableIHSNLP(m,s,d,initial,struct(),r);
assert(lifted.physicalNumZ==numel(physical) && numel(lifted.tube.lifting.cutTimes)==2,'ABFD:CovarianceMesh');
maxValue=0; maxContinuity=0;
for k=1:3
    point=physical+1e-6*sin((1:numel(physical)).'+k);
    z=buildAbfdCovarianceShootingSeed(lifted,point);
    [a,b,e]=ordinary.eval(point); [x,y,w]=lifted.eval(z);
    maxValue=max([maxValue,abs(full(a-x)),norm(full(b-y(1:numel(b))),inf),norm(full(e-w),inf)]);
    maxContinuity=max(maxContinuity,norm(full(y(numel(b)+1:end)),inf));
end
z=buildAbfdCovarianceShootingSeed(lifted,physical); np=numel(physical);
all=Function('abfd_shooting_partials',{lifted.z}, ...
    {jacobian(lifted.tube.lifting.equalities,lifted.z),jacobian([lifted.J;lifted.cIneq],lifted.z)});
[continuity,partial]=all(z); continuity=full(continuity); partial=full(partial);
Ga=continuity(:,np+1:end); assert(rcond(Ga)>1e-10,'ABFD:CovarianceTangentRank');
direction=sin((1:np).'); direction=direction/norm(direction);
auxiliaryDirection=-Ga\(continuity(:,1:np)*direction);
ordinaryDerivative=Function('abfd_original_covariance_tangent',{ordinary.z}, ...
    {jtimes([ordinary.J;ordinary.cIneq],ordinary.z,direction)});
expected=full(ordinaryDerivative(physical)); actual=partial*[direction;auxiliaryDirection];
tangentError=norm(expected-actual)/max(1,norm(expected));
directionAll=cos((1:numel(z)).'); directionAll=directionAll/norm(directionAll); h=1e-7;
f=Function('abfd_shooting_objective_constraints',{lifted.z},{[lifted.J;lifted.cIneq]});
finite=full(f(z+h*directionAll)-f(z-h*directionAll))/(2*h);
derivativeError=norm(partial*directionAll-finite)/max(1,norm(finite));
assert(maxValue<1e-8 && maxContinuity<1e-9 && tangentError<1e-8 && derivativeError<1e-5, ...
    'ABFD:CovarianceShootingContract');
coordinateValueError=0; coordinateRoundTrip=0;
if strcmp(coordinates,'symmetric')
    % Free symmetric moments remove only the factor-product nonlinearity.
    ga=jacobian(lifted.tube.lifting.equalities,lifted.z); ga=ga(:,np+1:end);
    nonlinear=logical(which_depends(ga,lifted.z,1,false));
    assert(~any(nonlinear(np+1:end)),'ABFD:CovarianceMomentAffine', ...
        'Moment continuity must be affine in all free covariance coordinates.');
    rr=r; rr.config=rmfield(rr.config,'covarianceCoordinates');
    factored=buildCasadiTrackableIHSNLP(m,s,d,initial,struct(),rr);
    for k=1:3
        point=physical+1e-6*sin((1:np).'+k);
        factorPoint=buildAbfdCovarianceShootingSeed(factored,point);
        factorPoint(np+1:end)=factorPoint(np+1:end)+1e-5*cos((1:numel(factorPoint)-np).'+k);
        momentPoint=convertAbfdSampledCovarianceCoordinates(factored.tube.lifting,np,factorPoint,'symmetric');
        recovered=convertAbfdSampledCovarianceCoordinates(lifted.tube.lifting,np,momentPoint,'cholesky');
        [fa,ga,ha]=factored.eval(factorPoint); [fb,gb,hb]=lifted.eval(momentPoint);
        coordinateValueError=max(coordinateValueError,max(abs(full([fa-fb;ga-gb;ha-hb]))));
        coordinateRoundTrip=max(coordinateRoundTrip,norm(recovered-factorPoint,inf));
    end
    bad=z; bad(np+1)=-1; rejected=false;
    try
        convertAbfdSampledCovarianceCoordinates(lifted.tube.lifting,np,bad,'cholesky');
    catch exception
        rejected=strcmp(exception.identifier,'ABFD:CovarianceCoordinatePositiveDefinite');
    end
    assert(rejected && coordinateValueError<1e-8 && coordinateRoundTrip<1e-10, ...
        'ABFD:CovarianceCoordinateContract','Off-continuity conversion or indefinite covariance refusal failed.');
end
report=struct('passed',true,'mode',mode,'coordinates',coordinates,'cuts',lifted.tube.lifting.cutTimes,'valueError',maxValue, ...
    'continuityResidual',maxContinuity,'implicitTangentRelativeError',tangentError, ...
    'partialDerivativeRelativeError',derivativeError,'fullVariables',np,'liftedVariables',numel(z), ...
    'coordinateValueError',coordinateValueError,'coordinateRoundTripError',coordinateRoundTrip, ...
    'auxiliaryJacobianReciprocalCondition',rcond(Ga), ...
    'scope','Exact sampled covariance propagation and implicit/full-space derivatives across off-clock knots; no optimization acceptance');
end
