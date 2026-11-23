function tr=c3TransportWarm(bundle,task,t)
% Preserve the previous route while satisfying changed stationary endpoints.
u=t/task.T; tr=sgReconstruct(bundle,u*bundle.T); ends=sgReconstruct(bundle,[0,bundle.T]);
d0=task.q0-ends.q(:,1); dg=task.qGoal-ends.q(:,2); delta=dg-d0;
s=10*u.^3-15*u.^4+6*u.^5; ds=(30*u.^2-60*u.^3+30*u.^4)/task.T;
dds=(60*u-180*u.^2+120*u.^3)/task.T^2;
tr.q=tr.q+d0+delta*s; tr.v=tr.v*(bundle.T/task.T)+delta*ds;
tr.a=tr.a*(bundle.T/task.T)^2+delta*dds; tr.t=t; tr.T=task.T;
% Previous forces remain a warm start; the new NLP enforces new dynamics.
end
