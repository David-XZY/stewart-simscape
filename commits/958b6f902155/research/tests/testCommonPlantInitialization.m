function report=testCommonPlantInitialization(executionRoot)
% Read-only replay of real independently recorded startup observations.
folders=dir(fullfile(executionRoot,'task_*')); assert(~isempty(folders),'ABFD:MissingStartupStudy');
rows=struct('taskId',{},'providerHash',{},'samePhysicalInitialization',{},'idempotent',{}, ...
    'differentForceRejected',{},'differentObjectRejected',{});
for k=1:numel(folders)
    directory=fullfile(folders(k).folder,folders(k).name);
    manifest=readAbfdJsonUtf8(fullfile(directory,'experiment','manifest.json'));
    data=load(fullfile(directory,'canonical_reference.mat'),'bundle'); bundle=data.bundle; c=manifest.config;
    changes=c.plantOverrides; if isfield(changes,'initialAppliedForce'), changes=rmfield(changes,'initialAppliedForce'); end
    shared=makeAbfdConfig(struct('plantProfile',c.plantProfile,'plantOverrides',changes, ...
        'plantInitializationFile',c.staticBalanceFile));
    coldProfile=makeHighFidelityPlantProfile(shared.plantProfile,shared.plantOverrides);
    [~,~,analyticProfile]=buildAbfdProviderForTask(bundle.model,shared,coldProfile,bundle.refs.q(:,1));
    c.plantInitializationFile=c.staticBalanceFile;
    [provider,c1,profile1]=buildAbfdProviderForTask(bundle.model,c,coldProfile,bundle.refs.q(:,1));
    [repeated,c2,profile2]=buildAbfdProviderForTask(bundle.model,c1,profile1,bundle.refs.q(:,1));
    assert(isequal(analyticProfile,profile1) && isequal(profile1,profile2) && ...
        strcmp(provider.hash,repeated.hash) && isequal(c1,c2),'ABFD:StartupComparison', ...
        'Analytic/B objects or repeated startup resolution differ.');
    changed=profile1; changed.initialAppliedForce(1)=changed.initialAppliedForce(1)+1e-6;
    forceRejected=reject(@()applyAbfdStaticPreload(changed,bundle,c.staticBalanceFile),'ABFD:PreloadInitialForce');
    changed=profile1; changed.viscousFriction=changed.viscousFriction+.01;
    objectRejected=reject(@()applyAbfdStaticPreload(changed,bundle,c.staticBalanceFile),'ABFD:PreloadContext');
    assert(forceRejected && objectRejected);
    rows(k)=struct('taskId',folders(k).name,'providerHash',provider.hash, ...
        'samePhysicalInitialization',true,'idempotent',true, ...
        'differentForceRejected',forceRejected,'differentObjectRejected',objectRejected);
end
report=struct('passed',true,'rows',rows, ...
    'scope','Real startup observations reused without fitting; analytic/B share the physical object and exact initial force, repeated configuration is identical, mismatches are rejected; no new simulation');
end
function rejected=reject(call,id)
rejected=false;
try
    call();
catch e
    assert(strcmp(e.identifier,id),'ABFD:UnexpectedRejection','Unexpected error: %s',e.identifier);
    rejected=true;
end
end
