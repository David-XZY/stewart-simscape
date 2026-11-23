function kernel=buildAbfdIntervalObserverKernel(provider)
% Compute the same RK4 map and every stage guard in one shared SX graph.
import casadi.*
ad=buildCasadiCompositeDynamics(provider); x=SX.sym('x',12); force=SX.sym('F',6); h=SX.sym('h');
k1=ad.f(x,force); x2=x+h/2*k1; k2=ad.f(x2,force);
x3=x+h/2*k2; k3=ad.f(x3,force); x4=x+h*k3; k4=ad.f(x4,force);
next=x+h/6*(k1+2*k2+2*k3+k4); stages=[x,x2,x3,x4];
mass=SX.zeros(6,24); transmission=SX.zeros(6,24); rate=SX.zeros(3,12);
for k=1:4
    [H,~,Jv]=ad.terms(stages(1:6,k),stages(7:12,k)); [~,E]=ad.geometry(stages(1:6,k));
    mass(:,6*(k-1)+(1:6))=H; transmission(:,6*(k-1)+(1:6))=Jv;
    rate(:,3*(k-1)+(1:3))=E;
end
outputs=cse({next,jacobian(next,x),stages,mass,transmission,rate});
kernel=Function('abfd_interval_observer_fused',{x,force,h},outputs);
end
