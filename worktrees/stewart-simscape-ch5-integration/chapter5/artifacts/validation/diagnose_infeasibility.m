function diagnose_infeasibility
[c,root]=ch5Init();import casadi.*
disp(CasadiMeta.version());
for k=5:5
 n=1;if k>1,n=12;end
 av=zeros(2,n);av(:,1)=1;a=DM(av);h=DM.eye(n);
 if k>=3,a=[DM.eye(n);DM.eye(n)];end
 options=struct('error_on_fail',false,'print_time',false,'osqp',struct('verbose',true,'max_iter',4000,'check_termination',1,'adaptive_rho',false,'rho',1,'eps_prim_inf',1e-3));
 f=conic('check_osqp','osqp',struct('h',h.sparsity(),'a',a.sparsity()),options);
 lo=[1;0];hi=lo;if k>=3,lo=[ones(n,1);zeros(n,1)];hi=lo;end
 bLo=-40*ones(n,1);bHi=40*ones(n,1);if k==4,bLo(1:6)=-inf;bHi(1:6)=inf;end;if k==5,bLo(1:6)=-1e30;bHi(1:6)=1e30;end
 ans=f('h',h,'g',zeros(n,1),'a',a,'lba',lo,'uba',hi,'lbx',bLo,'ubx',bHi);
 disp(k);disp(f.stats());disp(full(ans.x));
end
end



