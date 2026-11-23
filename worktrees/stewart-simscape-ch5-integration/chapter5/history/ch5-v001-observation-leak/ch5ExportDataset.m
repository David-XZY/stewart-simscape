function summary=ch5ExportDataset()
[c,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','teacher');summary=struct();expectedIdentity=ch5SourceIdentity();
taskIds=struct();
for split=["train","validation"]
    files=dir(fullfile(directory,split+'_*_force_baseline_0.mat'));contexts={};increments={};identities={};sources={};rejected={};
    expected=2*c.train_tasks;if split=="validation",expected=2*c.validation_tasks;end
    assert(numel(files)==expected,'ch5:IncompleteTeachers','%s teacher records: %d/%d.',split,numel(files),expected);
    seen={};
    for f=1:numel(files)
        sample=load(fullfile(files(f).folder,files(f).name),'result','records');
        assert(strcmp(sample.result.version,c.version)&&strcmp(sample.result.task.split,split));
        assert(isfield(sample.result,'source_identity')&&strcmp(sample.result.source_identity.baseline_source_sha256,expectedIdentity.baseline_source_sha256), ...
            'ch5:TeacherSourceMismatch','Teacher %s has a different baseline source identity.',files(f).name);
        assert(ismember(sample.result.status,{'completed','simulation_failed'}),'ch5:TeacherSetupFailure','A setup failure requires repair before training.');
        seen{end+1}=sample.result.task.id;
        if ~strcmp(sample.result.status,'completed')||~sample.result.metrics.safe_success
            rejected{end+1}=files(f).name;continue;
        end
        records=sample.records;count=numel(records);
        for k=1:c.window_stride:count-c.H+1
            event=cellfun(@(s)s.event,records(k:k+c.H-1));
            if any(event~=event(1)),continue;end
            visible=cell2mat(cellfun(@(s)s.observed(22,:)',records(k:k+c.H-1),'UniformOutput',false));
            if any(visible~=visible(:,1),'all'),continue;end
            u=cell2mat(cellfun(@(s)s.u,records(k:k+c.H-1),'UniformOutput',false));
            rec=records{k};du=diff([rec.previous_u,u],1,2);
            contexts{end+1}=ch5Context(rec.x,rec.previous_u,rec.goal,rec.observed,sample.result.mode);
            increments{end+1}=du(:);identities{end+1}=sample.result.task.id;
            sources{end+1}=struct('run',files(f).name,'task',sample.result.task.id,'mode',sample.result.mode, ...
                'first_control_index',k,'first_time',rec.time,'event',rec.event);
        end
    end
    assert(~isempty(contexts),'ch5:NoTrainingWindows','No successful physically valid %s runs supplied training windows.',split);
    context=cell2mat(contexts);delta=cell2mat(increments);file=fullfile(directory,split+'_windows.h5');
    taskIds.(split)=unique(seen);assert(numel(taskIds.(split))*2==expected,'ch5:TeacherTaskDuplicates','Incomplete task/profile coverage.');
    if isfile(file)
        assert(strcmp(h5readatt(file,'/','version'),c.version));
        assert(strcmp(h5readatt(file,'/','baseline_source_sha256'),expectedIdentity.baseline_source_sha256));
        assert(isequal(h5read(file,'/context'),single(context))&&isequal(h5read(file,'/delta'),single(delta)), ...
            'ch5:DatasetChanged','Existing dataset differs; preserve it and use a new dataset version.');
        summary.(split)=struct('runs',numel(files),'rejected_runs',numel(rejected),'windows',numel(contexts),'context_dimension',size(context,1));
        continue;
    end
    pending=[file,'.pending'];
    if isfile(pending),movefile(pending,[file,'.interrupted.',char(java.util.UUID.randomUUID())]);end
    h5create(pending,'/context',size(context),'Datatype','single');h5write(pending,'/context',single(context));
    h5create(pending,'/delta',size(delta),'Datatype','single');h5write(pending,'/delta',single(delta));
    h5writeatt(pending,'/','version',c.version);h5writeatt(pending,'/','window_stride',c.window_stride);
    h5writeatt(pending,'/','horizon',c.H);h5writeatt(pending,'/','split',char(split));
    h5writeatt(pending,'/','baseline_source_sha256',expectedIdentity.baseline_source_sha256);
    provenance=struct('identities',{identities},'rejected_runs',{rejected},'files',{string({files.name})}, ...
        'sources',{sources},'baseline_source_sha256',expectedIdentity.baseline_source_sha256,'task_ids',{taskIds.(split)});
    save(fullfile(directory,split+'_windows_provenance.mat'),'provenance');
    movefile(pending,file);
    summary.(split)=struct('runs',numel(files),'rejected_runs',numel(rejected),'windows',numel(contexts),'context_dimension',size(context,1));
end
assert(isempty(intersect(taskIds.train,taskIds.validation)),'ch5:SplitLeak','A complete task occurs in both splits.');
save(fullfile(directory,'dataset_summary.mat'),'summary');
manifest=struct('version',c.version,'baseline_source_sha256',expectedIdentity.baseline_source_sha256,'summary',summary,'raw_runs',[]);
files=dir(fullfile(directory,'*_force_baseline_0.mat'));raw=cell(numel(files),1);
for k=1:numel(files),raw{k}=struct('file',files(k).name,'sha256',hashFile(fullfile(files(k).folder,files(k).name)));end
manifest.raw_runs=vertcat(raw{:});
manifest.train_sha256=hashFile(fullfile(directory,'train_windows.h5'));manifest.validation_sha256=hashFile(fullfile(directory,'validation_windows.h5'));
fid=fopen(fullfile(directory,'dataset_manifest.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(manifest));fclose(fid);
end
function digest=hashFile(file)
fid=fopen(file,'rb');hash=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),hash.update(fread(fid,1048576,'*uint8'));end
fclose(fid);digest=lower(reshape(dec2hex(typecast(hash.digest(),'uint8'),2).',1,[]));
end
