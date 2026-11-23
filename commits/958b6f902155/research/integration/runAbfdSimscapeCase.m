function result=runAbfdSimscapeCase(bundle,provider,c,profile)
setup=prepareAbfdSimscape(bundle,provider,c,profile);
cleanup=onCleanup(@()close_system(setup.modelName,0));
accelerationInterface=installAbfdAccelerationProbe(setup.modelName);
simResult=sim(setup.modelName,'ReturnWorkspaceOutputs','on');
if isa(simResult,'Simulink.SimulationOutput'), simout=simResult.get('simout'); else, simout=simResult; end
log=readAbfdDirectAcceleration(simResult,readAbfdSimscapeLog(simout,setup));
tube=[];
if any(strcmp(c.controllerMode,{'geometric-D','fullstate-tvlqr'})) && ...
        ~any([c.qpEnabled,c.dobEnabled,c.governorEnabled,c.antiWindupEnabled])
    if isfield(c,'predictionProfile') && strcmp(c.predictionProfile,'actuator-var-v3')
        options=struct('queryTime',bundle.refs.t,'temporalFile',c.temporalFile,'allowPilotTemporal',false);
        if isfield(c,'allowPilotTemporal'), options.allowPilotTemporal=c.allowPilotTemporal; end
        if isfield(c,'predictionMaxStep'), options.maxStep=c.predictionMaxStep; end
        predicted=predictAbfdClosedLoopMean(setup.runtime,provider,profile,options);
        tube=struct('t',predicted.t,'P',predicted.P,'mean',predicted.x);
    else
        predicted=run_A0_trackability_metric(bundle.refs,provider,c);
        tube=struct('t',bundle.refs.t,'P',cat(3,predicted.P{:}));
        if isfield(predicted,'mean'), tube.mean=predicted.mean; end
    end
end
log.predictionError=zeros(12,numel(log.t)-1);
for k=1:numel(log.t)-1
    x=[log.q(:,k);log.qd(:,k)]; h=log.t(k+1)-log.t(k); F=log.actualForce(:,k);
    k1=provider.forward(x,F); k2=provider.forward(x+h/2*k1,F);
    k3=provider.forward(x+h/2*k2,F); k4=provider.forward(x+h*k3,F);
    estimate=x+h/6*(k1+2*k2+2*k3+k4);
    log.predictionError(:,k)=[log.q(:,k+1);log.qd(:,k+1)]-estimate;
end
metrics=evaluateUnifiedMetrics(log,bundle,c,tube);
acceptance=assessAbfdExperimentMetrics(metrics,c);
result=struct('log',log,'metrics',metrics,'profile',profile,'status','experiment-completed', ...
    'passed',acceptance.passed,'acceptance',acceptance,'interface',setup.interface, ...
    'accelerationInterface',accelerationInterface);
clear cleanup;
end
