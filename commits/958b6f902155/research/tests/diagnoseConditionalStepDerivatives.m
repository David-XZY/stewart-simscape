function report=diagnoseConditionalStepDerivatives(asset,bundle,calibrationFile,temporalFile)
import casadi.*
plain=makeHighFidelityPlantProfile('high-fidelity');
base=buildDynamicsProvider(bundle.model,'residual-B',asset,false,bundle.refs.q(:,1));
p=conditionResidualMeanOnStaticBalance(base,bundle,plain,calibrationFile);
profile=applyAbfdStaticPreload(plain,bundle,calibrationFile);
temporal=importResidualTemporalModel(temporalFile,asset,true,p);
step=buildAbfdPredictionStepAD(p,profile,temporal.standardizedMean);
force=profile.initialAppliedForce; target=force-profile.forceDeadzone;
z=[bundle.refs.q(:,1);zeros(6,1);target;force];
w=MX.sym('second_derivative_point',36); [A,B,G]=step.jacobian(w(1:24),w(25:30),w(31:36),.001);
first=reshape([A,B,G],24*36,1); evaluate=Function('step_first_derivative',{w},{first});
initial=[z;force;-temporal.standardizedMean]; steps=[1e-5,1e-6,1e-7,1e-8];
direction=sin((1:36).'); direction=direction/norm(direction); rows=struct([]);
for k=1:3
    if k==2, initial(13:18)=target+1e-12; end
    if k==3, direction(13:18)=direction(25:30); end
    derivative=Function('step_second_direction',{w},{jtimes(first,w,direction)});
    exact=full(derivative(initial)); errors=zeros(size(steps));
    for j=1:numel(steps)
        h=steps(j); finite=(full(evaluate(initial+h*direction))-full(evaluate(initial-h*direction)))/(2*h);
        errors(j)=norm(exact-finite)/max(1,norm(finite));
    end
    rows(k).consistentSlewTarget=(k==3); rows(k).roundoffOffset=(k>1); rows(k).errors=errors;
end
after=full(step.advance(initial(1:24),force,initial(31:36),0));
[Az,Bz,Gz]=step.jacobian(initial(1:24),force,initial(31:36),0);
zeroError=norm(after-initial(1:24))+norm(full(Az)-eye(24),'fro')+norm(full(Bz),'fro')+norm(full(Gz),'fro');
assert(zeroError<1e-10 && all(arrayfun(@(r)min(r.errors)<1e-7,rows)), ...
    'ABFD:StepDerivative','Zero-step identity or second derivative failed.');
report=struct('passed',true,'steps',steps,'cases',rows,'zeroStepError',zeroError, ...
    'scope','Completed slew-ramp and roundoff-scale second derivatives, zero-step identity');
end
