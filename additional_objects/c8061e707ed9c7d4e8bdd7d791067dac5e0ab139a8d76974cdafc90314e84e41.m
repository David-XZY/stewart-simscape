function report=ch5CheckDerivativeMap()
[c,root]=ch5Init();[m,s,ad,strict]=ch5Physics(c);import casadi.*
report=struct('version',c.version,'passed',false,'primal_tolerance',1e-9,'jacobian_tolerance',1e-4);
generator=RandStream('mt19937ar','Seed',7921);
for mode=["ideal-force","nonideal-force"]
    r=struct('c',c,'physical',m,'scene',s,'ad',ad,'strict',strict,'mode',mode);
    r.plant=ch5PlantModel(m,s,ch5ActuatorProfile(mode,c,m),struct('Ts',c.Ts,'substeps',8,'Np',c.H,'integralGridTs',0));
    fd=ch5DerivativeMap(r);xx=MX.sym('test_x',r.plant.n);uu=MX.sym('test_u',6);
    if mode=="ideal-force",yy=fd(xx,uu,zeros(6,1));else,yy=fd(xx,uu);end
    derivative=Function('ch5_checked_derivative',{xx,uu},{jacobian(yy,xx),jacobian(yy,uu)});
    nativeDerivative=Function('ch5_checked_native_derivative',{xx,uu},{jacobian(yy,xx),jacobian(yy,uu)},ch5ProjectionJitOptions());
    primal=0;stateError=0;inputError=0;oldInputError=0;memoryError=0;nativeError=0;
    for k=1:12
        q=m.qHome+[.02*randn(generator,3,1);.04*randn(generator,3,1)];
        [d,G]=ad.dynamics(q,zeros(6,1));equilibrium=full(G)\(-full(d));
        u=equilibrium;v=zeros(6,1);
        if k>2,u=u+40*randn(generator,6,1);v=.005*randn(generator,6,1);end
        x=[q;v];if mode=="nonideal-force",x=[x;repmat(equilibrium,3,1)];end
        if k>6&&mode=="nonideal-force",x(13:24)=x(13:24)+20*randn(generator,12,1);end
        original=full(r.plant.fd(x,u,zeros(6,1)));
        if mode=="ideal-force",actual=full(fd(x,u,zeros(6,1)));else,actual=full(fd(x,u));end
        primal=max(primal,norm(original-actual,inf));[A,B]=derivative(x,u);A=full(A);B=full(B);
        [nativeA,nativeB]=nativeDerivative(x,u);
        nativeError=max([nativeError;max(abs(full(nativeA)-A),[],'all');max(abs(full(nativeB)-B),[],'all')]);
        if mode=="nonideal-force",memoryError=max(memoryError,norm(original([13:18,25:30])-repmat(u,2,1),inf));end
        numericalA=zeros(size(A));numericalB=zeros(size(B));eps=.001;
        for col=1:r.plant.n
            h=eps;if col<=12,h=1e-6;end
            perturb=zeros(r.plant.n,1);perturb(col)=h;
            numericalA(:,col)=(full(r.plant.fd(x+perturb,u,zeros(6,1)))-full(r.plant.fd(x-perturb,u,zeros(6,1))))/(2*h);
        end
        for col=1:6
            perturb=zeros(6,1);perturb(col)=eps;
            numericalB(:,col)=(full(r.plant.fd(x,u+perturb,zeros(6,1)))-full(r.plant.fd(x,u-perturb,zeros(6,1))))/(2*eps);
        end
        stateError=max(stateError,max(abs(A-numericalA),[],'all'));inputError=max(inputError,max(abs(B-numericalB),[],'all'));
        [~,oldB]=r.plant.derivative(x,u);oldInputError=max(oldInputError,max(abs(full(oldB)-numericalB),[],'all'));
    end
    label=strrep(char(mode),'-','_');report.(label)=struct('primal_error',primal, ...
        'state_jacobian_error',stateError,'input_jacobian_error',inputError,'original_input_jacobian_error',oldInputError, ...
        'end_slew_and_memory_error',memoryError,'native_jacobian_error',nativeError,'cases',12);
    fprintf('Derivative map %s: primal %.3g, A %.3g, B %.3g, original B %.3g\n',mode,primal,stateError,inputError,oldInputError);
    assert(primal<=report.primal_tolerance&&stateError<=report.jacobian_tolerance&&inputError<=report.jacobian_tolerance&&memoryError<=1e-9&&nativeError<1e-8);
end
report.passed=true;save(fullfile(root,'chapter5','artifacts','validation','derivative_map.mat'),'report','-v7');
end
