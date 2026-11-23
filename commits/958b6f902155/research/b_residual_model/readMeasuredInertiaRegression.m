function measurement=readMeasuredInertiaRegression(calibrationFile,bundle,profile)
% Read independently measured world-acceleration response, never commanded force.
applyAbfdStaticPreload(profile,bundle,calibrationFile);
calibration=jsondecode(fileread(calibrationFile)); q=calibration.pose(:); S=calibration.sensitivity;
validateattributes(S,{'double'},{'size',[6,6],'real','finite'});
assert(rcond(S)>1e-8,'ABFD:InertiaSensitivity');
provider=buildDynamicsProvider(bundle.model); o=provider.evaluate(q,zeros(6,1),zeros(6,1));
M=o.Jv.'/S; [E,~]=rpyRateMapZYX(q(4:6),zeros(3,1)); T=blkdiag(eye(3),E);
symmetry=norm(M-M.','fro')/norm(M,'fro');
assert(symmetry<1e-8 && min(eig((M+M.')/2))>0,'ABFD:MeasuredSpatialMass');
zero=buildInertialWrenchRegressor(q,zeros(6,1),zeros(6,1),bundle.model); Y=zeros(36,10);
for k=1:6
    a=zeros(6,1); a(k)=1;
    Y(6*(k-1)+(1:6),:)=buildInertialWrenchRegressor(q,zeros(6,1),a,bundle.model)-zero;
end
% Put all entries in consistent mass units using the nominal platform radius.
lengthScale=bundle.model.rB; assert(lengthScale>0,'ABFD:InertiaLengthScale');
D=diag([ones(1,3),ones(1,3)/lengthScale]); scaled=zeros(36,10);
for k=1:10
    block=D*(reshape(Y(:,k),6,6)/T)*D; scaled(:,k)=block(:);
end
target=D*M*D; [~,R]=qr(scaled,0); assert(rcond(R)>1e-8,'ABFD:InertiaIdentificationRank');
parameters=Y\reshape(M*T,36,1); mass=parameters(1); com=parameters(2:4)/mass;
origin=[parameters(5),parameters(8),parameters(9);parameters(8),parameters(6),parameters(10); ...
    parameters(9),parameters(10),parameters(7)];
inertia=origin-mass*((com.'*com)*eye(3)-com*com.');
residual=norm(reshape(Y*parameters,6,6)-M*T,'fro')/norm(M*T,'fro');
assert(mass>0 && min(eig(inertia))>0 && min(eig(trace(inertia)/2*eye(3)-inertia))>=-1e-9 && ...
    norm(com)<.5 && residual<.01,'ABFD:InertiaRigidApproximation');
measurement=struct('design',scaled,'target',target(:),'mass',mass,'com',com,'inertia',inertia, ...
    'relativeRigidResidual',residual,'symmetryRelativeError',symmetry,'pose',q, ...
    'calibrationHash',abfdFileHash(calibrationFile),'nominalModelHash',abfdHash(bundle.model), ...
    'plantHash',calibration.plantHash,'lengthScale',lengthScale, ...
    'scope','Independent force/acceleration identification at one static pose, not closed-loop acceptance');
end
