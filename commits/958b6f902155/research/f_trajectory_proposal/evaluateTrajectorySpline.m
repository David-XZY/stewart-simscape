function [q,v,a]=evaluateTrajectorySpline(cp,time,duration)
% Clamped quintic B spline with ten control points. No spline toolbox needed.
assert(isequal(size(cp),[6,10]) && duration>0,'ABFD:SplineShape','Expected 6x10 quintic control points.');
u=min(max(time(:).'/duration,0),1); knots=[zeros(1,6),(.2:.2:.8),ones(1,6)];
q=cp*basis(knots,5,u); dcp=zeros(6,9);
for i=1:9, dcp(:,i)=5*(cp(:,i+1)-cp(:,i))/(knots(i+6)-knots(i+1)); end
dk=knots(2:end-1); v=dcp*basis(dk,4,u)/duration;
ddcp=zeros(6,8);
for i=1:8, ddcp(:,i)=4*(dcp(:,i+1)-dcp(:,i))/(dk(i+5)-dk(i+1)); end
a=ddcp*basis(dk(2:end-1),3,u)/duration^2;
end
function B=basis(knots,degree,u)
n=numel(knots)-degree-1; original=u; u(u==1)=1-eps;
B=zeros(numel(knots)-1,numel(u));
for i=1:size(B,1), B(i,:)=u>=knots(i) & u<knots(i+1); end
for p=1:degree
    next=zeros(size(B,1)-1,numel(u));
    for i=1:size(next,1)
        l=knots(i+p)-knots(i); r=knots(i+p+1)-knots(i+1);
        if l>0, next(i,:)=next(i,:)+(u-knots(i))/l.*B(i,:); end
        if r>0, next(i,:)=next(i,:)+(knots(i+p+1)-u)/r.*B(i+1,:); end
    end
    B=next;
end
B=B(1:n,:); B(:,original==1)=0; B(end,original==1)=1;
end
