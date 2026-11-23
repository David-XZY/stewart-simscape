function report=testIntervalObserver()
% Independent numeric integration and covariance checks; no trained fixtures.
m=buildOptModelCustom(); p=buildDynamicsProvider(m); x=[m.qHome;zeros(6,1)]; dt=.01;
F=p.inverse(x(1:6),x(7:12),zeros(6,1)); P=1e-4*eye(12); equilibrium=x;
for k=1:50
    [equilibrium,P]=stepAbfdIntervalObserver(equilibrium,P,x(1:6),dt*F,p,dt);
end
equilibriumError=norm(equilibrium-x,inf); assert(equilibriumError<1e-9);
x=x+[1e-3;-.5e-3;0;.001;-.001;.001;zeros(6,1)];
force=F+[1;-2;1;0;-1;1]; P=diag(1e-5*(1:12));
expected=advance(x,force); pose=expected(1:6);
[actual,covariance,predicted]=stepAbfdIntervalObserver(x,P,pose,dt*force,p,dt);
predictionError=norm(predicted-expected,inf); assert(predictionError<1e-10 && norm(actual-expected,inf)<1e-10);
J=zeros(12); h=1e-6;
for k=1:12
    direction=zeros(12,1); direction(k)=h;
    J(:,k)=(advance(x+direction,force)-advance(x-direction,force))/(2*h);
end
priorCov=J*P*J.'+dt*1e-6*eye(12);
[~,~,expectedCov]=stepAbfdExternalObserver(expected,priorCov,pose,zeros(6,1),p,0);
covarianceError=norm(covariance-expectedCov,'fro'); assert(covarianceError<1e-9 && min(eig(covariance))>0);
[initial,initialCov]=stepAbfdIntervalObserver(x,P,x(1:6),zeros(6,1),p,0);
assert(norm(initial-x)<1e-12 && min(eig(initialCov))>0);
rejected=false;
try
    stepAbfdIntervalObserver(x,P,pose,ones(6,1),p,0);
catch e
    rejected=strcmp(e.identifier,'ABFD:ObserverImpulseClock');
end
assert(rejected,'ABFD:ObserverTest','Impulse without elapsed time was accepted.');
report=struct('passed',true,'equilibriumError',equilibriumError,'predictionError',predictionError, ...
    'covarianceError',covarianceError,'scope','Numeric RK4 comparison, finite-difference covariance, equilibrium and causal impulse clock checks; no Simscape performance acceptance');
    function y=advance(value,input)
        k1=p.forward(value,input); k2=p.forward(value+dt/2*k1,input);
        k3=p.forward(value+dt/2*k2,input); k4=p.forward(value+dt*k3,input);
        y=value+dt/6*(k1+2*k2+2*k3+k4);
    end
end
