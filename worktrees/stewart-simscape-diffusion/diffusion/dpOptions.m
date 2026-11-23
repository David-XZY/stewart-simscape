function o=dpOptions(c)
o=makeCommonIpoptOptions(struct('maxIter',c.maxIter,'printLevel',c.printLevel, ...
    'tol',1e-9,'constrViolTol',1e-9,'acceptableIter',0,'hessianApproximation','exact'));
o.print_time=false; o.ipopt.max_cpu_time=c.maxCpuSeconds;
o.ipopt.bound_relax_factor=0;
end
