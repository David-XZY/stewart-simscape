function tr=dpReconstruct(b,t)
if strcmp(b.kind,'spline'), tr=dpSpline(b.cp,t,b.T); return; end
t=t(:).'; N=b.intervals; h=b.h;
q=zeros(6,numel(t)); v=q; a=q; F=q; stateV=q;
for j=1:numel(t)
    k=min(N,1+floor(t(j)/h)); u=(t(j)-(k-1)*h)/h;
    q0=b.X(1:6,k); q1=b.X(1:6,k+1); v0=b.X(7:12,k); v1=b.X(7:12,k+1);
    % Quintic Hermite reconstruction also preserves endpoint accelerations.
    % Under HS consistency the degree-five coefficient tends to zero.
    a0=b.A(:,k); a1=b.A(:,k+1);
    r0=q1-q0-h*v0-.5*h^2*a0; r1=h*(v1-v0)-h^2*a0; r2=h^2*(a1-a0);
    c3=10*r0-4*r1+.5*r2; c4=-15*r0+7*r1-r2; c5=6*r0-3*r1+.5*r2;
    q(:,j)=q0+h*v0*u+.5*h^2*a0*u^2+c3*u^3+c4*u^4+c5*u^5;
    v(:,j)=v0+h*a0*u+(3*c3*u^2+4*c4*u^3+5*c5*u^4)/h;
    a(:,j)=a0+(6*c3*u+12*c4*u^2+20*c5*u^3)/h^2;
    F(:,j)=2*(u-.5)*(u-1)*b.F(:,k)-4*u*(u-1)*b.F(:,N+1+k)+2*u*(u-.5)*b.F(:,k+1);
    stateV(:,j)=(2*u^3-3*u^2+1)*v0+(u^3-2*u^2+u)*h*b.A(:,k)+(-2*u^3+3*u^2)*v1+(u^3-u^2)*h*b.A(:,k+1);
end
tr=struct('t',t,'q',q,'v',v,'a',a,'F',F,'stateV',stateV,'T',b.T,'kind','reconstructed');
end
