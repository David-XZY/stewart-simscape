function profile=makeHighFidelityPlantProfile(mode,overrides)
if nargin<2, overrides=struct(); end
validatestring(mode,{'matched','high-fidelity'});
profile=struct('name',mode,'segmentMass',.001,'payloadMassScale',1,'payloadComOffset',zeros(3,1), ...
    'payloadInertiaScale',1,'axialStiffness',0,'axialDamping',0,'viscousFriction',0, ...
    'forceLag',.001,'forceDeadzone',0,'forceDelay',1e-6,'forceRate',8e5, ...
    'initialPoseOffset',zeros(6,1),'wrench',zeros(6,1));
if strcmp(mode,'high-fidelity')
    profile.segmentMass=1; profile.payloadMassScale=1.15;
    profile.payloadComOffset=[.01;-.01;.015]; profile.payloadInertiaScale=1.10;
    profile.axialStiffness=500; profile.axialDamping=20; profile.viscousFriction=10;
    profile.forceLag=.01; profile.forceDeadzone=5; profile.forceDelay=.002; profile.forceRate=4e4;
end
for f=fieldnames(overrides).'
    assert(isfield(profile,f{1}) || strcmp(f{1},'initialAppliedForce'),'ABFD:PlantOption','Unknown plant option.'); profile.(f{1})=overrides.(f{1});
end
assert(profile.segmentMass>0 && profile.payloadMassScale>0 && profile.payloadInertiaScale>0, ...
    'ABFD:PlantMass','Mass and inertia must be positive.');
for f={'axialStiffness','axialDamping','viscousFriction','forceDeadzone','forceDelay'}
    validateattributes(profile.(f{1}),{'double'},{'scalar','nonnegative','finite'});
end
if isfield(profile,'initialAppliedForce')
    validateattributes(profile.initialAppliedForce,{'double'},{'size',[6,1],'real','finite'});
end
end
