function [q,v,a]=ch5SpeedReferenceBatch(plan,time)
% Vectorized identical cubic Hermite reconstruction, including endpoint hold.
time=max(0,reshape(time,1,[]));hold=time>=plan.t(end);
k=sum(plan.t(:)<=time,1);k=max(1,min(k,numel(plan.t)-1));
h=plan.t(k+1)-plan.t(k);s=(time-plan.t(k))./h;
q=(2*s.^3-3*s.^2+1).*plan.q(:,k)+(s.^3-2*s.^2+s).*h.*plan.qd(:,k)+(-2*s.^3+3*s.^2).*plan.q(:,k+1)+(s.^3-s.^2).*h.*plan.qd(:,k+1);
v=(6*s.^2-6*s)./h.*plan.q(:,k)+(3*s.^2-4*s+1).*plan.qd(:,k)+(-6*s.^2+6*s)./h.*plan.q(:,k+1)+(3*s.^2-2*s).*plan.qd(:,k+1);
a=(12*s-6)./h.^2.*plan.q(:,k)+(6*s-4)./h.*plan.qd(:,k)+(-12*s+6)./h.^2.*plan.q(:,k+1)+(6*s-2)./h.*plan.qd(:,k+1);
q(:,hold)=repmat(plan.q(:,end),1,nnz(hold));v(:,hold)=0;a(:,hold)=0;
end
