function [timed,log]=p474Retime(path,task,clockHandle,budget,c,previous)
% Fixed geometry, fixed T. Solver failure never certifies infeasibility.
import casadi.*
timer=tic; n=c.retimeControls; sgrid=linspace(0,1,81); [B,D,DD]=bdBasis(sgrid*task.T,task.T,n);
z=MX.sym('time_free',n-6); d=[zeros(3,1);z;ones(3,1)];
f=bdPathFunction(path); [Q,Qs,Qss]=f(d.'*B); V=Qs.*repmat(d.'*D,6,1); A=Qss.*repmat((d.'*D).^2,6,1)+Qs.*repmat(d.'*DD,6,1);
p=pgPhysics(task.model); m=task.model; [L,Ld,Ldd,F,~,~,phi]=p.point(Q,V,A);
g=[diff(d);L(:);Ld(:);Ldd(:);F(:)]; count=numel(sgrid);
lo=[zeros(n-1,1);repmat(m.lmin,count,1);repmat(-m.actuator.ldotMax,count,1);repmat(-m.actuator.lddotMax,count,1);repmat(m.actuator.forceMin,count,1)];
hi=[inf(n-1,1);repmat(m.lmax,count,1);repmat(m.actuator.ldotMax,count,1);repmat(m.actuator.lddotMax,count,1);repmat(m.actuator.forceMax,count,1)];
objective=sum2(sum1((Ldd/1.2).^2))+.01*sum2(sum1((F/2000).^2))+.01*sum2(phi);
cc=c; cc.maxIter=c.retimeIterations; cc.maxCpuSeconds=max(.001,budget-toc(clockHandle));
options=dpOptions(cc); callback=bdDeadline(numel(z),numel(g),0); callback.clockHandle=clockHandle; callback.budget=budget; options.iteration_callback=callback; solver=nlpsol('bd_retime','ipopt',struct('x',z,'f',objective,'g',g),options);
timed=struct('kind','path_time_map','path',path,'pathHash',dpHash(path),'T',task.T,'timeControls',[]);
log=struct('status','deadline','solverSuccess',false,'seconds',0,'geometryPreserved',true);
if toc(clockHandle)<budget
    initial=linspace(0,1,n-4); initial=initial(2:end-1); if ~isempty(previous),initial=previous(4:end-3);end; sol=solver('x0',initial,'lbx',zeros(n-6,1),'ubx',ones(n-6,1),'lbg',lo,'ubg',hi);
    stats=solver.stats(); timed.timeControls=[zeros(1,3),full(sol.x).',ones(1,3)];
    log.status=char(stats.return_status); log.solverSuccess=stats.success;
end
log.interrupted=strcmp(log.status,'deadline')||contains(log.status,'Maximum_Iterations')||contains(log.status,'Maximum_CpuTime')||contains(log.status,'User_Requested_Stop');
log.seconds=toc(timer); log.withinBudget=toc(clockHandle)<=budget;
end
