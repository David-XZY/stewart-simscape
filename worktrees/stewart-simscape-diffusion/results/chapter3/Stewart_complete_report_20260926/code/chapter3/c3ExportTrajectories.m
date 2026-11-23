function c3ExportTrajectories(runName)
c=c3Setup(runName); folder=fullfile(c.output,'report','trajectories'); if ~isfolder(folder), mkdir(folder); end
files=dir(fullfile(c.output,'jobs','*','result.mat'));
for k=1:numel(files)
    z=load(fullfile(files(k).folder,files(k).name),'row'); r=z.row;
    if ~r.passed || r.repeat~=1 || ~strcmp(r.purpose,'main') || ~endsWith(r.caseId,'_01') || strcmp(r.category,'sequence'), continue; end
    file=fullfile(folder,[r.jobId,'.json']); if isfile(file), continue; end
    z=load(fullfile(files(k).folder,files(k).name)); task=z.item.task;
    t=linspace(0,task.T,ceil(task.T/.005)+1); tr=sgReconstruct(z.out.bundle,t); p=sgPhysics(task.model);
    [L,Ld,Ldd,F]=p.point(tr.q,tr.v,tr.a);
    data=struct('jobId',r.jobId,'caseId',r.caseId,'method',r.method,'family',r.family,'time',t, ...
        'q',tr.q,'v',tr.v,'a',tr.a,'L',full(L),'Ld',full(Ld),'Ldd',full(Ldd),'FRequired',full(F), ...
        'auditTime',z.audit.time,'clearance',z.audit.clearance,'obstacles',task.scene.hood.obstacles, ...
        'object',task.scene.objectCylinder,'limits',task.model.actuator,'score',r.score,'quality',r.quality);
    dpWriteJson(file,data);
end
fprintf('C3_TRAJECTORIES_EXPORTED\n');
end
