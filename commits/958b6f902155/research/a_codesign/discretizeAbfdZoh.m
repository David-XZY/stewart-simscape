function [A,G]=discretizeAbfdZoh(Ac,Gc,dt)
% Differentiable frozen-linear ZOH; feedback is applied at discrete ticks.
% Fixed scaling/order avoid decision-dependent CasADi branches.
n=size(Ac,1); h=dt/16; Z=Ac*h; term=eye(n); A=term; integral=term;
for k=1:16
    term=term*Z/k; A=A+term; integral=integral+term/(k+1);
end
integral=h*integral;
for k=1:4
    integral=integral+A*integral; A=A*A;
end
G=integral*Gc;
end
