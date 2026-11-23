function report=ch5CheckRepairNumerics()
% A real infeasible QP must not turn an OSQP certificate into force changes.
[c,root]=ch5Init();import casadi.*
y=MX.sym('repair_y',12);p=MX.sym('repair_p',18);
g=[y;y];J=jacobian(g,y);Q=DM.eye(12);
linear=Function('repair_linearization',{y,p},{g,.5*dot(y,y),y,J});
solver=conic('repair_infeasible_qp','osqp',struct('h',Q.sparsity(),'a',J.sparsity()), ...
    struct('error_on_fail',false,'print_time',false,'osqp',struct('verbose',false,'max_iter',c.projection_qp_max_iterations, ...
        'check_termination',25,'adaptive_rho_interval',50, ...
        'eps_abs',c.projection_qp_tolerance,'eps_rel',c.projection_qp_tolerance)));
optimizer=struct('kind','sqp_osqp','H',1,'n',12,'x_scale',ones(12,1), ...
    'g_scale',ones(24,1),'lower',[ones(12,1);zeros(12,1)],'upper',[ones(12,1);zeros(12,1)], ...
    'input_trust_region',40,'linearization',linear,'quadratic_hessian',Q,'solver',solver);
answer=ch5SolveProjection(optimizer,zeros(12,1),zeros(18,1),[]);
assert(contains(lower(string(answer.stats.return_status)),'infeasible'));
assert(answer.rejected_solver_output&&all(isfinite(answer.x))&&norm(answer.x,inf)==0);
report=struct('version',c.version,'control_revision',c.control_revision, ...
    'infeasible_status',answer.stats.return_status,'certificate_rejected',answer.rejected_solver_output, ...
    'osqp_infinity_sentinel',1e30,'finite_held_decision',all(isfinite(answer.x)));
% A former plan shifted by two samples must still be a usable initial guess.
[m,s,ad,strict]=ch5Physics(c);r=struct('c',c,'physical',m,'scene',s,'ad',ad,'strict',strict,'mode',"ideal-force");
r.plant=ch5PlantModel(m,s,ch5ActuatorProfile(r.mode,c,m),struct('Ts',c.Ts,'substeps',8,'Np',c.H,'integralGridTs',0));
q=m.qHome+[0;0;.15;zeros(3,1)];[d,G]=ad.dynamics(q,zeros(6,1));u=full(G)\(-full(d));x=[q;zeros(6,1)];
xx=MX.sym('repair_initial',12);uu=MX.sym('repair_inputs',6,c.H);nodes=xx;current=xx;
for j=1:c.H,current=r.plant.fd(current,uu(:,j),zeros(6,1));nodes=[nodes,current];end
preview=Function('repair_state_preview',{xx,uu},{nodes});
warm=struct('U',repmat(u,1,c.H),'artificial_force',u,'shift_steps',2);
[U,X,info]=ch5ControlInitialGuess(x,u,warm,r,preview);
assert(strcmp(info.source,'shifted_plan')&&info.shift_steps==2&&all(isfinite(X),'all')&&norm(U-warm.U,inf)<1e-10);
report.two_sample_warm_retained=true;report.passed=true;
save(fullfile(root,'chapter5','artifacts','validation','repair_numerics.mat'),'report','-v7');disp(report);
end
