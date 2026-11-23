function cp=dpBaseline(task)
t=linspace(0,task.T,201); u=t/task.T;
[B,~,~]=dpBasis(t,task.T); q=task.q0+(task.qGoal-task.q0)*(10*u.^3-15*u.^4+6*u.^5);
cp=zeros(6,16); cp(:,1:3)=repmat(task.q0,1,3); cp(:,14:16)=repmat(task.qGoal,1,3);
cp(:,4:13)=(q-cp*B)/B(4:13,:);
end
