function exportCompactDiffusion()
root='D:/NPU/Matlab_code/stewart-simscape-diffusion';
destination='D:/NPU/Matlab_code/output/diffusion_package_20260917';
addpath(fullfile(root,'diffusion'),fullfile(root,'study')); setupDiffusion();
pools={}; bundles={}; records={}; poolKeys=containers.Map(); bundleKeys=containers.Map();
studyRoot=fullfile(root,'results','study','frozen_v1'); j=load(fullfile(studyRoot,'jobs.mat'));
for k=1:numel(j.jobs)
    job=j.jobs{k}; folder=fullfile(studyRoot,'jobs',job.jobId);
    row=jsondecode(fileread(fullfile(folder,'result.json')));
    r=entry('frozen_v1',job.jobId,row.passed); r.status=row.status;
    if isfile(fullfile(folder,'result.mat')), file=fullfile(folder,'result.mat');
    elseif isfile(fullfile(folder,'online.mat')), file=fullfile(folder,'online.mat');
    else, file=''; end
    if ~isempty(file)
        s=load(file,'out'); r.source=strrep(file,[root,'/'],'');
        if ~isempty(s.out)
            r.candidatePool=pool(s.out.candidates); r.best=bundle(s.out.bundle); r.first=bundle(s.out.firstBundle);
            r.events=zeros(1,numel(s.out.eventBundles));
            for e=1:numel(r.events), r.events(e)=bundle(s.out.eventBundles{e}); end
        end
    else
        file=fullfile(studyRoot,'regenerated_candidates',[job.jobId,'.mat']);
        if isfile(file)
            s=load(file,'candidates'); r.candidatePool=pool(s.candidates); r.regenerated=true; r.source=file;
        end
    end
    records{end+1}=r; %#ok<AGROW>
end
for name={'pilot_v4','formal_v1'}
    files=dir(fullfile(root,'results','remote','diffusion',name{1},'benchmark','*','*','benchmark.mat'));
    for k=1:numel(files)
        file=fullfile(files(k).folder,files(k).name); s=load(file,'row','candidates','outcome','task');
        r=entry(name{1},[s.row.taskId,'__',s.row.method],s.row.passed); r.source=file;
        r.candidatePool=pool(s.candidates);
        if ~isempty(s.outcome) && ~isempty(s.outcome.best), r.best=bundle(s.outcome.best.bundle); end
        records{end+1}=r; %#ok<AGROW>
    end
end
files=dir(fullfile(root,'results','remote','diffusion','formal_v1_fastpath_v2','*','measurement.mat'));
for k=1:numel(files)
    file=fullfile(files(k).folder,files(k).name); s=load(file);
    r=entry('formal_v1_fastpath_v2',s.task.taskId,s.row.finalPassed); r.source=file; r.candidatePool=pool(s.candidates);
    if ~isempty(s.acceptedCp), r.best=bundle(struct('kind','spline','cp',s.acceptedCp,'T',s.task.T));
    elseif ~isempty(s.fallback) && s.fallback.passed, r.best=bundle(s.fallback.best.bundle); end
    records{end+1}=r; %#ok<AGROW>
end
files=dir(fullfile(studyRoot,'reference_checkpointed_v1','*','checkpoint.mat'));
for k=1:numel(files)
    file=fullfile(files(k).folder,files(k).name); s=load(file,'checkpoint'); [~,id]=fileparts(files(k).folder);
    r=entry('reference_checkpointed_v1',id,true); r.source=file;
    r.candidatePool=pool(s.checkpoint.candidates); r.best=bundle(s.checkpoint.bundle); r.first=bundle(s.checkpoint.firstBundle);
    for e=1:numel(s.checkpoint.eventBundles), r.events(e)=bundle(s.checkpoint.eventBundles{e}); end
    r.status='accepted_checkpoint_only_see_original_completion_status'; records{end+1}=r; %#ok<AGROW>
end
for k=1:numel(records), records{k}.source=strrep(strrep(records{k}.source,'\','/'),[root,'/'],''); end
metadata=struct('schemaVersion',1,'numericType','double, no rounding or downsampling', ...
    'records',numel(records),'candidatePools',numel(pools),'trajectoryBundles',numel(bundles), ...
    'note','Exact candidate controls and trajectory reconstruction coefficients. Pool and bundle IDs are MATLAB one-based; zero means unavailable. Dense audits remain in original workspace; scalar reports are packaged. Regenerated controls never imply successful planning.');
file=fullfile(destination,'compact_trajectories.mat'); save(file,'records','pools','bundles','metadata','-v7');
roundtrip=load(file); assert(isequaln(roundtrip.pools,pools) && isequaln(roundtrip.bundles,bundles));
assert(isequaln(roundtrip.records,records));
dpWriteJson(fullfile(destination,'compact_trajectory_index.json'),struct('metadata',metadata,'records',{records}));
fprintf('COMPACT_EXPORT records=%d pools=%d bundles=%d roundtrip=PASS\n',numel(records),numel(pools),numel(bundles));
    function r=entry(stage,id,passed)
        r=struct('stage',stage,'id',id,'originalPassed',passed,'status','','source','', ...
            'candidatePool',0,'best',0,'first',0,'events',[],'regenerated',false);
    end
    function id=pool(cp)
        id=0; if isempty(cp), return; end
        key=dpHash(cp); if isKey(poolKeys,key), id=poolKeys(key); return; end
        pools{end+1}=cp; id=numel(pools); poolKeys(key)=id;
    end
    function id=bundle(b)
        id=0; if isempty(b), return; end
        if isfield(b,'separators'), b=rmfield(b,'separators'); end
        key=dpHash(b); if isKey(bundleKeys,key), id=bundleKeys(key); return; end
        bundles{end+1}=b; id=numel(bundles); bundleKeys(key)=id;
    end
end
