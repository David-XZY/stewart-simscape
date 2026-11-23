function [B,D,DD]=dpBasis(t,T)
% Clamped quintic spline, 16 controls, analytic first and second derivatives.
assert(T>0 && all(isfinite(t(:))),'DP:Time','Invalid trajectory clock.');
u=max(0,min(1,t(:).'/T)); knots=[zeros(1,6),(1:10)/11,ones(1,6)];
B=basis(knots,5,u); G=zeros(15,16);
for i=1:15, G(i,i:i+1)=[-5,5]/(knots(i+6)-knots(i+1)); end
dk=knots(2:end-1); H=zeros(14,15);
for i=1:14, H(i,i:i+1)=[-4,4]/(dk(i+5)-dk(i+1)); end
D=G.'*basis(dk,4,u)/T;
DD=G.'*H.'*basis(dk(2:end-1),3,u)/T^2;
end
function B=basis(knots,p,u)
last=u==1; u(last)=1-eps;
B=zeros(numel(knots)-1,numel(u));
for i=1:size(B,1), B(i,:)=u>=knots(i) & u<knots(i+1); end
for d=1:p
    C=zeros(size(B,1)-1,numel(u));
    for i=1:size(C,1)
        l=knots(i+d)-knots(i); r=knots(i+d+1)-knots(i+1);
        if l>0, C(i,:)=C(i,:)+(u-knots(i))/l.*B(i,:); end
        if r>0, C(i,:)=C(i,:)+(knots(i+d+1)-u)/r.*B(i+1,:); end
    end
    B=C;
end
B(:,last)=0; B(end,last)=1;
end
