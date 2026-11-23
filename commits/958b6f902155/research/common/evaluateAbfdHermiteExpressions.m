function reference=evaluateAbfdHermiteExpressions(X,A,t,query)
% Exact symbolic restriction of the same C2 quintics used for execution.
% X=[q;qdot] at all HS nodes/midpoints, A=qddot at those points.
t=t(:).'; query=query(:).';
assert(all(diff(t)>0) && all(query>=t(1)-1e-12 & query<=t(end)+1e-12), ...
    'ABFD:HermiteQuery','Reference queries must remain inside increasing knots.');
assert(isequal(size(X),[12,numel(t)]) && isequal(size(A),[6,numel(t)]), ...
    'ABFD:HermiteShape','Invalid position/derivative knot arrays.');
values=cell(1,numel(query));
for j=1:numel(query)
    k=find(t<=query(j)+1e-12,1,'last'); k=min(k,numel(t)-1);
    h=t(k+1)-t(k); s=(query(j)-t(k))/h;
    c0=X(1:6,k); c1=h*X(7:12,k); c2=h^2*A(:,k)/2;
    rhs=[X(1:6,k+1)-c0-c1-c2,h*X(7:12,k+1)-c1-2*c2,h^2*A(:,k+1)-2*c2];
    coeff=[c0,c1,c2,rhs/[1,3,6;1,4,12;1,5,20]];
    q=coeff*[1;s;s^2;s^3;s^4;s^5];
    v=coeff(:,2:6)*[1;2*s;3*s^2;4*s^3;5*s^4]/h;
    a=coeff(:,3:6)*[2;6*s;12*s^2;20*s^3]/h^2;
    values{j}=[q;v;a];
end
reference=[values{:}];
end
