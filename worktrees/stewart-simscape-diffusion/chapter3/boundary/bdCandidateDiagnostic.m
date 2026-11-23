function bdCandidateDiagnostic(jobsFile,runName)
c=bdConfig(runName); protocol=jsondecode(fileread(jobsFile)); jobs=protocol.jobs;
asset=bdLoadLibrary(protocol.libraryFile,'train',c);
for k=1:numel(jobs)
    if iscell(jobs), job=jobs{k}; else, job=jobs(k); end
    task=bdMakeTask(job,job.parameters,c); local=asset;
    if isfield(job,'modelFile') && ~isempty(job.modelFile), local.model=jsondecode(fileread(job.modelFile)); end
    clock=tic; pool=bdCandidates(job.method,task,1:job.K,local,clock,60,c,job.samplingSeed);
    rows={};
    for j=1:numel(pool)
        if toc(clock)>=60, break; end
        [ok,r]=bdAccept(pool{j}.bundle,task,'physical',clock,60,c);
        row=struct('id',pool{j}.meta.id,'passed',ok,'reason',r.reason,'modeFeature',[]);
        if ok
            tr=bdReconstruct(pool{j}.bundle,linspace(0,task.T,33)); feature=zeros(6,33);
            for v=1:33
                R=rpy2rotmZYX(tr.q(4:6,v)); feature(:,v)=[(tr.q(1:3,v)+R*task.scene.objectCylinder.center_P)/.1;(R*task.scene.objectCylinder.axis_P)/.2];
            end
            row.modeFeature=feature(:).';
        end
        rows{end+1}=row;
    end
    result=struct('job',job,'generated',numel(pool),'checked',numel(rows),'seconds',toc(clock),'candidates',{rows},'repair',false,'fallback',false);
    folder=fullfile(c.output,'candidate_diagnostics'); if ~isfolder(folder), mkdir(folder); end
    dpWriteJson(fullfile(folder,[job.runId,'.json']),result);
end
end
