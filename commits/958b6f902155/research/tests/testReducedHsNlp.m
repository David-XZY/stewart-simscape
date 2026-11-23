function report=testReducedHsNlp(asset,useShooting,mode)
% Check equivalence on off-solution motions, bounds and complete derivatives.
% This short fixture is deliberately not a feasible task or training asset.
import casadi.*
if nargin<2, useShooting=false; end
if nargin<3, mode='global-qr'; end
validateattributes(useShooting,{'logical'},{'scalar'});
m=buildOptModelCustom(); s=buildCylinderBoxTransferScene(m);
s.q0=m.qHome; s.qWaypoint=m.qHome; s.qGoal=m.qHome;
s.qd0=zeros(6,1); s.qdGoal=zeros(6,1);
s.phase.durationApproach=.03; s.phase.durationInsertion=.015;
d=buildTwoPhaseIHSDiscretization(s,2,1); [~,initial]=buildInitialGuessTwoPhaseIHSImplicit(m,s,d);
p=buildDynamicsProvider(m); modelMode='analytic';
if nargin>0 && ~isempty(asset), p=buildDynamicsProvider(m,'residual-B',asset); modelMode='residual-B'; end
initial.Xnode=repmat([m.qHome;zeros(6,1)],1,d.numNodes);
initial.XmidIHS=repmat([m.qHome;zeros(6,1)],1,d.numMidpoints);
initial.Anode=zeros(6,d.numNodes); initial.Amid=zeros(6,d.numMidpoints);
force=p.inverse(m.qHome,zeros(6,1),zeros(6,1));
initial.Fnode=repmat(force,1,d.numNodes); initial.Fmid=repmat(force,1,d.numMidpoints);
z=packIHSDecisionImplicit(initial.Xnode(:,2:end-1),initial.XmidIHS, ...
    initial.Anode,initial.Amid,initial.Fnode,initial.Fmid,d,initial.separator);
c=makeAbfdConfig(struct('predictionProfile','sampled-hold-v2','controllerMode','geometric-D', ...
    'dVariant','fixed','designMode','codesign-A','modelMode',modelMode));
if useShooting
    c.plannerProfile='guarded-exact-v1'; c.solverFormulation='covariance-shooting-hs-reduced';
    c.covarianceShootingInterval=.02; c=makeAbfdConfig(c);
end
raw=struct('t',d.tNode,'tc',d.tMid,'Xnode',initial.Xnode,'Xmid',initial.XmidIHS, ...
    'Anode',initial.Anode,'Amid',initial.Amid);
refs=reconstructIhsidContinuousReference(raw,p,c.sampleTime);
design=buildAbfdSchedule(refs,p,c); design.nominal=refs;
research=struct('provider',p,'config',c,'design',design,'center',z,'buildOnly',true,'prepareReduction',true);
original=buildCasadiTrackableIHSNLP(m,s,d,initial,struct(),research);
if useShooting
    z=buildAbfdCovarianceShootingSeed(original,z);
    assert(numel(original.tube.lifting.cutTimes)==2,'ABFD:CombinedReductionMesh','The full shooting state must remain present.');
end
reduced=buildAbfdReducedHsNlp(original,d,z,mode);
if strcmp(mode,'local-midpoint')
    assert(reduced.report.retainedHsEquations==6*d.numIntervals && ...
        reduced.report.eliminatedHsRank==18*d.numIntervals && ...
        max(sum(spones(reduced.motionBasis),2))<=4, ...
        'ABFD:LocalHsContract','Keep every inter-node position equation and preserve interval-local dependence.');
end
maxEquality=0; maxComparison=0; maxRoundTrip=0;
for k=1:5
    value=reduced.x0+1e-5*sin((1:numel(reduced.x0)).'+k); lifted=full(reduced.lift(value));
    [jo,eo,io]=original.eval(lifted); [jr,er,ir]=reduced.eval(value);
    eo=full(eo); io=full(io); er=full(er); ir=full(ir);
    maxEquality=max(maxEquality,norm(eo(reduced.removedEquality),inf));
    expected=[io;original.lbz(reduced.lowerBoundIndices)-lifted(reduced.lowerBoundIndices); ...
        lifted(reduced.upperBoundIndices)-original.ubz(reduced.upperBoundIndices)];
    maxComparison=max([maxComparison,abs(full(jo-jr)),norm(er-eo(reduced.retainedEquality),inf),norm(ir-expected,inf)]);
    maxRoundTrip=max(maxRoundTrip,norm(full(reduced.lift(lifted(reduced.keepIndices)))-lifted,inf));
end
direction=sin((1:numel(reduced.x0)).'); direction=direction/norm(direction);
y=reduced.nlp.x; expression=[reduced.nlp.f;reduced.nlp.g];
derivative=Function('abfd_reduced_direction',{y},{jtimes(expression,y,direction)});
value=reduced.x0+2e-5*direction; h=1e-7;
f=Function('abfd_reduced_all',{y},{expression}); finite=full(f(value+h*direction)-f(value-h*direction))/(2*h);
exact=full(derivative(value)); derivativeError=norm(exact-finite)/max(1,norm(finite));
assert(maxEquality<1e-8 && maxComparison<1e-9 && maxRoundTrip<1e-10 && derivativeError<1e-5, ...
    'ABFD:ReducedHsContract','Reduced/full values, eliminated equations, bounds or derivatives differ.');
report=struct('passed',true,'modelMode',modelMode,'covarianceShooting',useShooting,'eliminatedEqualityResidual',maxEquality, ...
    'comparisonError',maxComparison,'roundTripError',maxRoundTrip,'derivativeRelativeError',derivativeError, ...
    'reduction',reduced.report,'scope','Off-solution reduced/full-space equivalence and bound/AD checks; no trajectory or optimization acceptance');
assert(~isempty(jsonencode(report)),'ABFD:ReductionReport','The audit must be serializable on the supported MATLAB version.');
end
