function [A,G]=discretizeAbfdClosedLoop(Ac,Gc,dt)
% Fourth-order frozen-linear discretization; identical in NLP and diagnostics.
Z=Ac*dt; Z2=Z*Z; Z3=Z2*Z; Z4=Z3*Z;
A=eye(12)+Z+Z2/2+Z3/6+Z4/24;
G=dt*(eye(12)+Z/2+Z2/6+Z3/24)*Gc;
end
