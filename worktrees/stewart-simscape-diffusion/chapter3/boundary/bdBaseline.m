function cp=bdBaseline(task,n)
if nargin<2, n=16; end
if n==16, cp=dpBaseline(task); return; end
s=linspace(0,1,401); [B,~,~]=bdBasis(s,1,n); h=10*s.^3-15*s.^4+6*s.^5;
cp=[repmat(task.q0,1,3),zeros(6,n-6),repmat(task.qGoal,1,3)];
cp(:,4:n-3)=(task.q0+(task.qGoal-task.q0)*h-cp*B)/B(4:n-3,:);
end
