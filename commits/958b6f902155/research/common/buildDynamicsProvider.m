function p = buildDynamicsProvider(model, mode, asset, allowFixture, initialPose, startup)
% W=[world force; world moment about platform origin], x=[ZYX q;qdot].
if nargin<2, mode='analytic'; end
if nargin<3, asset=''; end
if nargin<4, allowFixture=false; end
if nargin<5, initialPose=[]; end
if nargin<6, startup=[]; end
validatestring(mode,{'analytic','residual-B'});
net=[]; effective=model; elasticity=[];
if strcmp(mode,'residual-B')
    if isstruct(asset), net=validateResidualNetwork(asset,allowFixture);
    else, net=importResidualNetworkWeights(asset,allowFixture); end
    physical=net.physical;
    if isfield(net,'meanAnchoring')
        assert(isstruct(startup) && isscalar(startup) && isfield(startup,'nominalModelHash') && ...
            strcmp(startup.nominalModelHash,abfdHash(model)) && numel(initialPose)==6 && ...
            norm(startup.pose(:)-initialPose(:),inf)<1e-10, ...
            'ABFD:StartupInertiaContextRequired','Anchored B requires independently measured inertia for this model and initial pose.');
        validateattributes(startup.mass,{'double'},{'scalar','real','finite','positive'});
        validateattributes(startup.com,{'double'},{'numel',3,'real','finite'});
        validateattributes(startup.inertia,{'double'},{'size',[3,3],'real','finite'});
        assert(startup.relativeRigidResidual<.01 && startup.relativeRigidResidual>=0 && ...
            startup.symmetryRelativeError<1e-8 && startup.symmetryRelativeError>=0 && norm(startup.com)<.5 && ...
            ~isempty(regexp(startup.calibrationHash,'^[0-9a-f]{64}$','once')) && ...
            ~isempty(regexp(startup.plantHash,'^[0-9a-f]{64}$','once')), ...
            'ABFD:StartupInertiaEvidence','Invalid independent startup measurement.');
        physical.mass=startup.mass; physical.com=startup.com(:); physical.inertia=startup.inertia;
    elseif isfield(net,'referencePhysical')
        % Transfer identified inertial-parameter increments, not an absolute payload.
        old=net.referencePhysical;
        mass=model.dynamics.totalMass+physical.mass-old.mass;
        moment=model.dynamics.totalMass*model.dynamics.comP+physical.mass*physical.com(:)-old.mass*old.com(:);
        shift=@(r) (r.'*r)*eye(3)-r*r.';
        origin=model.dynamics.inertiaAtCOM_P+model.dynamics.totalMass*shift(model.dynamics.comP)+ ...
            physical.inertia+physical.mass*shift(physical.com(:))-old.inertia-old.mass*shift(old.com(:));
        physical.mass=mass; physical.com=moment/mass; physical.inertia=origin-mass*shift(physical.com);
    end
    assert(physical.mass>0 && norm(physical.inertia-physical.inertia.','fro')<1e-9 && ...
        min(eig(physical.inertia))>0 && ...
        min(eig(trace(physical.inertia)/2*eye(3)-physical.inertia))>=-1e-9, ...
        'ABFD:TransferredInertia','Transferred parameters are not physically realizable.');
    effective.dynamics.totalMass=physical.mass;
    effective.dynamics.comP=physical.com(:);
    effective.dynamics.inertiaAtCOM_P=physical.inertia;
    if isfield(net.physical,'elasticity')
        assert(~isempty(initialPose),'ABFD:ResidualContextRequired', ...
            'This structured B asset requires the initial pose for its spring rest lengths.');
        validateattributes(initialPose,{'double'},{'real','finite','numel',6});
        rest=sgpIK(initialPose(:),model); elasticity=net.physical.elasticity;
        elasticity.referencePose=initialPose(:); elasticity.referenceLength=rest.L;
    end
end
assert(isempty(startup) || (~isempty(net) && isfield(net,'meanAnchoring')), ...
    'ABFD:UnexpectedStartupInertia','This asset does not declare startup inertia conditioning.');
p=struct('mode',mode,'model',effective,'nominalModel',model,'network',net, ...
    'hash',abfdHash(struct('model',effective,'network',net)), ...
    'frame','world wrench at moving-platform origin; ZYX generalized coordinates');
if ~isempty(elasticity)
    p.elasticity=elasticity;
    p.hash=abfdHash(struct('model',effective,'network',net,'elasticity',elasticity));
end
if ~isempty(startup)
    p.parameterCalibration=startup;
    p.hash=abfdHash(struct('baseProviderHash',p.hash,'parameterCalibration',startup));
end
p.evaluate=@(q,v,a) evaluate(q,v,a,effective,net,elasticity);
p.forward=@(x,F) forward(x,F,effective,net,elasticity);
p.inverse=@(q,v,a) inverse(q,v,a,effective,net,elasticity);
end
function out=evaluate(q,v,a,model,net,elasticity)
q=q(:); v=v(:); a=a(:);
validateattributes([q;v;a],{'double'},{'real','finite','numel',18});
assert(abs(cos(q(5)))>1e-5,'ABFD:RpySingular','RPY pitch is singular.');
[~,aux]=stateDynamicsCompositeRigidBody([q;v],zeros(6,1),model);
assert(rcond(aux.Jout.Jv.')>model.num.rcondMin,'ABFD:TransmissionSingular','Ill-conditioned transmission.');
mu=zeros(6,1); covariance=zeros(6);
if ~isempty(net), [mu,covariance]=evaluateResidualMlp([q;v],net); end
if ~isempty(elasticity)
    kin=sgpIK(q,model);
    mu=mu+aux.Jout.Jv.'*(elasticity.stiffness*(kin.L-elasticity.referenceLength)+ ...
        elasticity.damping*(aux.Jout.Jq*v));
end
out=struct('H',aux.H,'bias',aux.Wbias+mu,'Jv',aux.Jout.Jv, ...
    'Jq',aux.Jout.Jq,'W',aux.H*a+aux.Wbias+mu,'Sigma',covariance);
out.Gamma=out.Jv.'\out.H;
end
function F=inverse(q,v,a,m,n,e)
requireReady(n);
o=evaluate(q,v,a,m,n,e); F=o.Jv.'\o.W;
end
function xd=forward(x,F,m,n,e)
requireReady(n);
o=evaluate(x(1:6),x(7:12),zeros(6,1),m,n,e);
xd=[x(7:12);o.H\(o.Jv.'*F(:)-o.bias)];
end
function requireReady(net)
assert(isempty(net) || ~isfield(net,'meanAnchoring'),'ABFD:StartupBalanceRequired', ...
    'Complete independent static-balance conditioning before control or prediction.');
end
