function report=diagnoseConditionalCovarianceDerivatives(asset,bundle,calibrationFile,temporalFile)
import casadi.*
plain=makeHighFidelityPlantProfile('high-fidelity');
base=buildDynamicsProvider(bundle.model,'residual-B',asset,false,bundle.refs.q(:,1));
p=conditionResidualMeanOnStaticBalance(base,bundle,plain,calibrationFile);
profile=applyAbfdStaticPreload(plain,bundle,calibrationFile);
temporal=importResidualTemporalModel(temporalFile,asset,true,p);
t=[0,.015,.03]; X=repmat([bundle.refs.q(:,1);zeros(6,1)],1,3); A=zeros(6,3);
X(1,2:3)=X(1,2:3)+[2e-5,5e-5]; X(5,2:3)=X(5,2:3)+[1e-5,2e-5];
X(7,2:3)=[.001,.002]; X(11,2:3)=[.0005,.001];
raw=struct('t',t([1,3]),'tc',t(2),'Xnode',X(:,[1,3]),'Xmid',X(:,2),'Anode',A(:,[1,3]),'Amid',A(:,2));
refs=reconstructIhsidContinuousReference(raw,p,.01); rows=struct([]);
for mode={'fullstate-tvlqr','geometric-D'}
    c=makeAbfdConfig(struct('modelMode','residual-B','residualFile',asset,'controllerMode',mode{1}, ...
        'predictionProfile','actuator-var-v3','temporalFile',temporalFile,'allowPilotTemporal',true));
    design=buildAbfdSchedule(refs,p,c); symbol=MX.sym('covariance_decision',54); knots=reshape(symbol,18,3);
    rollout=buildAbfdActuatorRolloutAD(knots(1:12,:),knots(13:18,:),t,p,design,c,profile,temporal,.001,true);
    output=[];
    count=numel(rollout.trace.P);
    for k=1:count, output=[output;reshape(rollout.trace.P{k},3600,1)]; end %#ok<AGROW>
    evaluate=Function('all_query_covariances',{symbol},{output}); direction=sin((1:54).'); direction=direction/norm(direction);
    derivative=Function('all_query_directions',{symbol},{jtimes(output,symbol,direction)});
    initial=[X;A]; initial=initial(:); exact=full(derivative(initial)); h=1e-8;
    finite=(full(evaluate(initial+h*direction))-full(evaluate(initial-h*direction)))/(2*h);
    differences=reshape(exact-finite,60,60,count); errors=zeros(count,4); largest=zeros(count,3);
    for k=1:count
        truth=reshape(finite(3600*(k-1)+(1:3600)),60,60); d=differences(:,:,k);
        regions={1:12,13:24,25:36,37:60};
        for j=1:4
            ids=regions{j}; errors(k,j)=norm(d(ids,ids),'fro')/max(1,norm(truth(ids,ids),'fro'));
        end
        [v,index]=max(abs(d(:))); [i,j]=ind2sub([60,60],index); largest(k,:)=[v,i,j];
    end
    stateFunction=Function('trace_inputs',{symbol},{[rollout.trace.input{:}]});
    row=struct('controller',mode{1},'time',rollout.trace.time,'step',rollout.trace.step, ...
        'input',full(stateFunction(initial)),'blockErrors',errors,'largestEntry',largest);
    if isempty(rows), rows=row; else, rows(end+1)=row; end %#ok<AGROW>
end
report=struct('cases',rows,'scope','Locate covariance derivative error by time and physical/actuator/command/VAR blocks');
end
