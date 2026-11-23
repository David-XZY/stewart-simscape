function [F,info]=stepGeometricTvlqrController(time,x,refs,schedule,provider)
i=find(refs.t<=time,1,'last'); i=min(i,numel(refs.t));
ref=struct('q',interp1(refs.t,refs.q.',time,'linear').', ...
    'qd',interp1(refs.t,refs.qd.',time,'linear').','qdd',interp1(refs.t,refs.qdd.',time,'linear').');
K=schedule.K(:,:,min(i,size(schedule.K,3)));
[F,info]=computeGeometricInverseDynamicsForce(x,ref,K,provider,schedule.variant);
end
