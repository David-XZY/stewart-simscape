function report=testIntervalObserverKernel(model)
% Compare the candidate kernel to the deployed observer without changing it.
if nargin<1, model=buildOptModelCustom(); end
p=buildDynamicsProvider(model); base=[model.qHome;zeros(6,1)];
force=p.inverse(base(1:6),base(7:12),zeros(6,1)); P=diag(1e-5*(1:12));
maximumState=0; maximumPrediction=0; maximumCovariance=0;
times=[0,.001,.01,.015]; checks=0;
for k=1:12
    x=base+1e-3*sin(k+(1:12).'); measurement=x(1:6)+1e-5*cos(k+(1:6).');
    F=force+5*cos(k+(1:6).');
    for dt=times
        [a,Pa,xa]=stepAbfdIntervalObserver(x,P,measurement,dt*F,p,dt);
        [b,Pb,xb]=stepAbfdIntervalObserverFused(x,P,measurement,dt*F,p,dt);
        maximumState=max(maximumState,norm(a-b,inf));
        maximumPrediction=max(maximumPrediction,norm(xa-xb,inf));
        maximumCovariance=max(maximumCovariance,norm(Pa-Pb,inf)); checks=checks+1;
    end
end
assert(max([maximumState,maximumPrediction,maximumCovariance])<1e-10, ...
    'ABFD:ObserverKernelEquivalence','Prediction, corrected state or covariance changed.');
% Refusals include a singular intermediate RK stage, not only the input pose.
singular=base; singular(5)=pi/2;
intermediate=base; intermediate(11)=2*(pi/2-intermediate(5))/.01;
badP=P; badP(1,1)=-1;
cases={{base,P,base(1:6),ones(6,1),0}, ...
       {base,badP,base(1:6),.01*force,.01}, ...
       {singular,P,base(1:6),.01*force,.01}, ...
       {intermediate,P,base(1:6),.01*force,.01}};
for k=1:numel(cases)
    item=cases{k}; oldRejected=false; newRejected=false;
    try
        stepAbfdIntervalObserver(item{1},item{2},item{3},item{4},p,item{5});
    catch
        oldRejected=true;
    end
    try
        stepAbfdIntervalObserverFused(item{1},item{2},item{3},item{4},p,item{5});
    catch
        newRejected=true;
    end
    assert(oldRejected && newRejected,'ABFD:ObserverKernelRefusal','Both implementations must reject invalid case %d.',k);
end
report=struct('passed',true,'comparisons',checks,'refusals',numel(cases), ...
    'maximumCorrectedStateDifference',maximumState,'maximumPredictionDifference',maximumPrediction, ...
    'maximumCovarianceDifference',maximumCovariance,'modelHash',p.hash, ...
    'scope','Analytic-model observer kernel equivalence and singular-stage refusals; not timing or Simscape acceptance');
end
