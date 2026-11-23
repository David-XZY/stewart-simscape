function [Q,V,A,F,Am,Fm]=ch5SpeedIhsidGuess(old,age,q0,v0,goal,T,r)
N=60;grid=linspace(0,T,N+1);[Q,V,A]=ch5SpeedReferenceBatch(old,age+grid);
s=grid/T;b=10*s.^3-15*s.^4+6*s.^5;bd=(30*s.^2-60*s.^3+30*s.^4)/T;
bdd=(60*s-180*s.^2+120*s.^3)/T^2;
e0=q0-Q(:,1);e1=goal-Q(:,end);ev=v0-V(:,1);
Q=Q+e0*(1-b)+e1*b+ev*(grid.*(1-s).^3);
V=V+(e1-e0)*bd+ev*((1-s).^2.*(1-4*s));
A=A+(e1-e0)*bdd+ev*((-6+18*s-12*s.^2)/T);
F=full(r.ad.feedforward(Q,V,A));h=T/N;Am=(A(:,1:end-1)+A(:,2:end))/2;
Xm=([Q(:,1:end-1);V(:,1:end-1)]+[Q(:,2:end);V(:,2:end)])/2+h/8*([V(:,1:end-1);A(:,1:end-1)]-[V(:,2:end);A(:,2:end)]);
Fm=full(r.ad.feedforward(Xm(1:6,:),Xm(7:12,:),Am));
end
