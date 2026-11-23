function report=testActuatorPrediction()
% Independent numerical controller-provider/RK4 against the AD actuator step.
m=buildOptModelCustom(); p=buildDynamicsProvider(m); profile=makeHighFidelityPlantProfile('high-fidelity');
x=[m.qHome;.01*ones(6,1)]; F=p.inverse(x(1:6),x(7:12),zeros(6,1));
z=[x;F+[0;20;-30;10;40;-10];F+[5;-5;10;-10;15;-15]];
u=F+[30;-600;800;40;-30;900]; h=.0025;
ad=buildAbfdPredictionStepAD(p,profile); value=full(ad.advance(z,u,zeros(6,1),h));
numeric=numericStep(z,u,h,p,profile);
numericError=norm(value-numeric); assert(numericError<1e-9);
[A,B,~]=ad.jacobian(z,u,zeros(6,1),h); exact=[full(A),full(B)]; finite=zeros(24,30);
for j=1:30
    delta=zeros(30,1); delta(j)=1e-5;
    finite(:,j)=(numericStep(z+delta(1:24),u+delta(25:30),h,p,profile)- ...
        numericStep(z-delta(1:24),u-delta(25:30),h,p,profile))/(2e-5);
end
jacobianError=norm(exact-finite,'fro')/norm(finite,'fro');
assert(jacobianError<1e-6,'ABFD:ActuatorStepDerivative','Actuator/body AD derivative mismatch.');
target=sign(u).*max(abs(u)-profile.forceDeadzone,0); z(13:18)=target;
[A,B]=ad.jacobian(z,u,zeros(6,1),h);
assert(norm(full(A(13:18,13:18)),'fro')<1e-12 && norm(full(B(13:18,:))-eye(6),'fro')<1e-12);
% Constant-target partition invariance of the exact actuator, including ramp completion.
state=struct('slew',F,'lag',F); [whole,fWhole]=advanceAbfdActuator(state,target,.031,profile,m);
part=advanceAbfdActuator(state,target,.004,profile,m);
[part,fPart]=advanceAbfdActuator(part,target,.027,profile,m);
partitionError=norm([whole.slew-part.slew;whole.lag-part.lag;fWhole-fPart]); assert(partitionError<1e-9);
report=struct('passed',true,'numericStepError',numericError,'stepJacobianRelativeError',jacobianError, ...
    'actuatorPartitionError',partitionError,'completedRampDerivativePassed',true);
end
function next=numericStep(z,u,h,p,profile)
x=z(1:12); state=struct('slew',z(13:18),'lag',z(19:24));
target=sign(u).*max(abs(u)-profile.forceDeadzone,0);
[~,f0]=advanceAbfdActuator(state,target,0,profile,p.model);
[~,fm]=advanceAbfdActuator(state,target,h/2,profile,p.model);
[last,f1]=advanceAbfdActuator(state,target,h,profile,p.model);
k1=p.forward(x,f0); k2=p.forward(x+h*k1/2,fm); k3=p.forward(x+h*k2/2,fm); k4=p.forward(x+h*k3,f1);
next=[x+h*(k1+2*k2+2*k3+k4)/6;last.slew;last.lag];
end
