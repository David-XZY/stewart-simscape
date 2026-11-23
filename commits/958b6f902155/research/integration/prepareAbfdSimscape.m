function setup=prepareAbfdSimscape(bundle,provider,c,profile)
root=setupAbfdResearch();
if nargin<4, profile=makeHighFidelityPlantProfile(c.plantProfile); end
runtime=makeAbfdRuntime(bundle,provider,c);
modelName='stewart_platform_model'; modelFile=fullfile(root,'matlab',[modelName,'.slx']);
assert(~bdIsLoaded(modelName),'ABFD:ModelAlreadyLoaded','Close the existing loaded Stewart model before preparing a research run.');
addpath(fullfile(root,'matlab')); addpath(fullfile(root,'matlab','simscape_subsystems'));
data=buildSimscapeLengthControlData(bundle.model,bundle.scene,'nonideal-force');
data.stewart=initializeCylindricalStruts(data.stewart,'Fsm',profile.segmentMass,'Msm',profile.segmentMass);
data.stewart.actuators.K=profile.axialStiffness*ones(6,1);
data.stewart.actuators.C=(profile.axialDamping+profile.viscousFriction)*ones(6,1);
qinitial=bundle.refs.q(:,1)+profile.initialPoseOffset;
if isfield(provider,'elasticity')
    assert(norm(provider.elasticity.referencePose-qinitial,inf)<1e-10,'ABFD:ResidualContextMismatch', ...
        'Controller spring rest lengths do not match the plant initial pose.');
end
data.stewart=initializeStewartPose(data.stewart,'AP',qinitial(1:3)-bundle.model.qHome(1:3), ...
    'ARB',rpy2rotmZYX(qinitial(4:6)));
data.payload.m=data.payload.m*profile.payloadMassScale;
data.payload.center=data.payload.center+profile.payloadComOffset;
data.payload.I=data.payload.I*profile.payloadInertiaScale;
data.disturbances=initializeDisturbances('Fd',profile.wrench,'Fd_t',0);
act=data.stewart.nonidealForceActuator; act.timeConstant=profile.forceLag;
act.deadzone=profile.forceDeadzone*ones(6,1); act.inputDelay=profile.forceDelay;
act.forceRateLimit=profile.forceRate*ones(6,1);
% Physical actuator initialization must not change when B replaces the
% controller-side model on a fixed reference. Analytic runs keep their value.
initialProvider=buildDynamicsProvider(bundle.model);
act.initialForce=initialProvider.inverse(bundle.refs.q(:,1),bundle.refs.qd(:,1),bundle.refs.qdd(:,1));
if isfield(profile,'initialAppliedForce'), act.initialForce=profile.initialAppliedForce; end
assert(all(act.initialForce>=bundle.model.actuator.forceMin & act.initialForce<=bundle.model.actuator.forceMax), ...
    'ABFD:InitialForceBounds','Physical initial force exceeds actuator limits.');
runtime.initialAppliedForce=act.initialForce;
data.stewart.nonidealForceActuator=act;
refs=bundle.refs; t=refs.t(:); origin=bundle.model.qHome;
legSpeed=zeros(6,numel(t)); jacobian=zeros(36,numel(t));
for k=1:numel(t)
    J=sgpJacobian(refs.q(:,k),bundle.model); legSpeed(:,k)=J.Jq*refs.qd(:,k); jacobian(:,k)=J.Jq(:);
end
references=struct('r',timeseries((refs.q-origin).',t),'rL',timeseries((refs.L-refs.L(:,1)).',t), ...
    'uFF',timeseries(refs.Fleg.',t),'uCT',timeseries(refs.Fleg.',t), ...
    'rd',timeseries(refs.qd.',t),'rdd',timeseries(refs.qdd.',t),'rLd',timeseries(legSpeed.',t), ...
    'rJq',timeseries(jacobian.',t));
vars=struct('stewart',data.stewart,'payload',data.payload,'ground',data.ground, ...
    'disturbances',data.disturbances,'references',references,'controller',initializeController('type','open-loop'), ...
    'K',ss(zeros(6)),'abfdRuntime',runtime,'computedTorqueConfig',makeSimscapePoseForceConfig(bundle.model), ...
    'computedTorqueModel',bundle.model);
for f=fieldnames(vars).', assignin('base',f{1},vars.(f{1})); end
evalin('base',sprintf('load(''%s'');',strrep(fullfile(root,'mat','conf_simscape.mat'),'''','''''')));
load_system(modelFile); set_param(modelName,'StopTime',num2str(t(end),16),'SimulationMode','normal');
configureSimscapeGravity(modelName,data.gravity,'enabled',true);
interface=installAbfdSimscapeRuntime(modelName);
interface.observerKernel='interval-rk4-v1';
if isfield(c,'observerKernel'), interface.observerKernel=c.observerKernel; end
runtime.initialState=[qinitial;zeros(6,1)]; assignin('base','abfdRuntime',runtime);
setup=struct('modelName',modelName,'modelFile',modelFile,'runtime',runtime,'interface',interface, ...
    'profile',profile,'modelSaved',false);
end
