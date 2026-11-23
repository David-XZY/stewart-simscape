function runtime=makeAbfdRuntime(bundle,provider,c)
assert(bundle.publishPassed && strcmp(bundle.providerHash,provider.hash), ...
    'ABFD:ModelVersionMismatch','Reference and controller must use the same provider.');
runtime=struct('bundle',bundle,'provider',provider,'config',c,'poseOrigin',bundle.model.qHome, ...
    'initialState',[bundle.refs.q(:,1);bundle.refs.qd(:,1)],'design',[],'lqi',[]);
runtime.governor=makeAttitudeCommandGovernorConfig(c.sampleTime,2,1,[2;2;3],[20;20;30]);
runtime.strict=[];
if c.qpEnabled
    assert(strcmp(c.modelMode,'analytic') && numel(bundle.scene.hood.obstacles)==3, ...
        'ABFD:QpCompatibility','Legacy strict-QP ablation supports the analytic model and the original three OBBs only.');
    runtime.strict=makeStrictClfCbfQpConfig(bundle.model,struct('dt',c.sampleTime));
end
assert(~c.antiWindupEnabled || any(strcmp(c.controllerMode,{'baseline-lqi','scheduled-lqi'})), ...
    'ABFD:AntiWindupCompatibility','Integral anti-windup requires an LQI controller.');
if any(strcmp(c.controllerMode,{'baseline-lqi','scheduled-lqi'}))
    nominalProvider=buildDynamicsProvider(bundle.model);
    ad=buildCasadiCompositeDynamics(nominalProvider); % fixed nominal feedback across B ablation
    refs=bundle.refs; n=1; if strcmp(c.controllerMode,'scheduled-lqi'), n=numel(refs.t); end
    gains=zeros(6,18,n); spectralRadius=zeros(1,n); cfg=makeSimscapePoseForceConfig(bundle.model);
    dt=c.sampleTime; leak=2*pi*cfg.lqiIntegralLeakHz;
    C=[eye(6),zeros(6)];
    scale=[cfg.lqiTranslationScale*ones(3,1);cfg.lqiRotationScale*ones(3,1)];
    intscale=[cfg.lqiIntegralScale*ones(3,1);cfg.lqiIntegralScale/bundle.model.Lc*ones(3,1)];
    Q=blkdiag(C.'*diag(1./scale.^2)*C+1e-9*eye(12),diag(1./intscale.^2));
    for k=1:n
        nominalForce=nominalProvider.inverse(refs.q(:,k),refs.qd(:,k),refs.qdd(:,k));
        [A,B]=ad.linearize([refs.q(:,k);refs.qd(:,k)],nominalForce);
        % The S-function holds leg force for dt and updates the leaky integral
        % with the current position error. Design for exactly that recurrence.
        discrete=expm([full(A),full(B);zeros(6,18)]*dt);
        augmentedA=[discrete(1:12,1:12),zeros(12,6);dt*C,(1-dt*leak)*eye(6)];
        augmentedB=[discrete(1:12,13:18);zeros(6)];
        [gains(:,:,k),~,poles]=dlqr(augmentedA,augmentedB,Q,eye(6)/cfg.lqiControlScale^2);
        spectralRadius(k)=max(abs(poles));
        assert(spectralRadius(k)<1,'ABFD:LqiDiscreteStability','Discrete LQI design is not locally stable.');
    end
    runtime.lqi=struct('gains',gains,'leak',leak,'spectralRadius',spectralRadius, ...
        'definition','ZOH discrete full-state LQI with explicit leaky integral and existing design weights');
else
    runtime.design=buildAbfdSchedule(bundle.refs,provider,c);
end
end
