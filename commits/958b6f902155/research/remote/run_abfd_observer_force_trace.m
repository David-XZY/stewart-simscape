function report=run_abfd_observer_force_trace(frozenFile,outputDir,observerVersion,duration)
% Short continuous linear-plant fixture using the deployed controller S-function.
% The staged S-function only adds an Update-method input/command trace.
if nargin<3, observerVersion='legacy'; end
if nargin<4, duration=.15; end
observerVersion=validatestring(observerVersion,{'legacy','interval'});
assert(~isfolder(outputDir),'ABFD:TraceExists','Use a fresh diagnostic output.'); mkdir(outputDir);
data=load(frozenFile,'tasks'); index=find(strcmp({data.tasks.split},'train'),1); task=data.tasks(index);
m=task.model; p=buildDynamicsProvider(m); dt=.01; q=m.qHome;
F=p.inverse(q,zeros(6,1),zeros(6,1));
c=makeAbfdConfig(struct('controllerMode','baseline-lqi','stateSource','external-observer','sampleTime',dt));
b=struct('publishPassed',true,'providerHash',p.hash,'model',m,'scene',task.scene, ...
    'refs',struct('q',repmat(q,1,2),'qd',zeros(6,2),'qdd',zeros(6,2),'Fleg',repmat(F,1,2),'t',[0,duration]));
r=makeAbfdRuntime(b,p,c); ad=buildCasadiCompositeDynamics(p); [A,B]=ad.linearize([q;zeros(6,1)],F);
lag=.001; plantA=[full(A),full(B);zeros(6,12),-eye(6)/lag]; plantB=[zeros(12,6);eye(6)/lag];
initialError=zeros(18,1); initialError(1)=1e-6;
vars=struct('abfdRuntime',r,'tracePlantA',plantA,'tracePlantB',plantB,'traceInitialError',initialError, ...
    'traceNominalForce',F,'traceVelocityMap',blkdiag(eye(3),rpyRateMapZYX(q(4:6))));
for field=fieldnames(vars).', assignin('base',field{1},vars.(field{1})); end
global ABFD_OBSERVER_TRACE; ABFD_OBSERVER_TRACE=[];
global ABFD_OBSERVER_STATE_TRACE; ABFD_OBSERVER_STATE_TRACE=[];
name='abfd_observer_force_trace'; assert(~bdIsLoaded(name),'ABFD:TraceLoaded','Close the previous trace model first.');
new_system(name); cleanup=onCleanup(@()close_system(name,0));
add_block('simulink/Continuous/State-Space',[name,'/Plant'],'A','tracePlantA','B','tracePlantB', ...
    'C','eye(18)','D','zeros(18,6)','X0','traceInitialError');
add_block('simulink/Signal Routing/Demux',[name,'/Sensors'],'Outputs','[6 6 6]');
add_block('simulink/Math Operations/Gain',[name,'/World Velocity'],'Gain','traceVelocityMap','Multiplication','Matrix(K*u)');
add_block('simulink/Sources/Constant',[name,'/Nominal Force'],'Value','traceNominalForce');
add_block('simulink/Math Operations/Sum',[name,'/Force Error'],'Inputs','+-');
add_block('simulink/Math Operations/Sum',[name,'/Measured Force'],'Inputs','++');
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name,'/Controller'], ...
    'FunctionName','geometricTvlqrSimscapeSfun','Parameters','abfdRuntime');
add_line(name,'Plant/1','Sensors/1'); add_line(name,'Sensors/1','Controller/1');
add_line(name,'Sensors/2','World Velocity/1'); add_line(name,'World Velocity/1','Controller/2');
add_line(name,'Sensors/3','Measured Force/1'); add_line(name,'Nominal Force/1','Measured Force/2');
add_line(name,'Measured Force/1','Controller/3'); add_line(name,'Controller/1','Force Error/1');
add_line(name,'Nominal Force/1','Force Error/2'); add_line(name,'Force Error/1','Plant/1');
outputs=6;
if strcmp(observerVersion,'interval')
    add_block('simulink/Continuous/Integrator',[name,'/Measured Force Integral'],'InitialCondition','zeros(6,1)');
    add_line(name,'Measured Force/1','Measured Force Integral/1'); add_line(name,'Measured Force Integral/1','Controller/4');
    outputs=7;
end
for k=2:outputs
    add_block('simulink/Sinks/Terminator',sprintf('%s/Output%d',name,k));
    add_line(name,sprintf('Controller/%d',k),sprintf('Output%d/1',k));
end
set_param(name,'Solver','ode15s','StopTime',num2str(duration),'MaxStep','.0005','RelTol','1e-8','AbsTol','1e-11');
sim(name); trace=ABFD_OBSERVER_TRACE; states=ABFD_OBSERVER_STATE_TRACE;
clear global ABFD_OBSERVER_TRACE ABFD_OBSERVER_STATE_TRACE;
assert(size(trace,1)==19 && size(trace,2)>=10,'ABFD:MissingObserverTrace','Missing native observer input trace.');
actual=trace(2:7,2:end); current=trace(8:13,2:end); previous=trace(14:19,2:end);
previousRms=norm(actual-previous,'fro')/sqrt(numel(actual)); currentRms=norm(actual-current,'fro')/sqrt(numel(actual));
report=struct('state','complete','observerVersion',observerVersion,'taskId',task.taskId,'sampleTime',dt,'forceLag',lag,'samples',size(trace,2), ...
    'measuredVersusPreviousCommandRmsN',previousRms,'measuredVersusCurrentCommandRmsN',currentRms, ...
    'inputMatchesPreviousCommand',previousRms<.02*currentRms, ...
    'sfunctionHash',abfdFileHash(which('geometricTvlqrSimscapeSfun')), ...
    'scope','Continuous linear training-model plant, 1 ms force lag, exact deployed S-function plus trace only; identifies force timing in its Update method. No Simscape or performance acceptance.');
if ~isempty(states)
    report.peakTranslation= max(sqrt(sum((states(2:4,:)-q(1:3)).^2,1)));
    report.finalTranslation=norm(states(2:4,end)-q(1:3));
    report.peakStateEstimateError=max(abs(states(2:13,:)-states(14:25,:)),[],2);
end
save(fullfile(outputDir,'trace.mat'),'trace','states','report','task'); writeAbfdJson(fullfile(outputDir,'status.json'),report);
clear cleanup;
end
