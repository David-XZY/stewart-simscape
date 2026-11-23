function report=exportStartupRawObservations(datasetFile,replayRoot,outputFile)
% Export logged channels unchanged for independent NumPy reconstruction.
assert(~isfile(outputFile),'ABFD:RawObservationsExist','Use a fresh audit artifact.');
d=jsondecode(fileread(datasetFile));
report=struct('datasetHash',abfdFileHash(datasetFile),'scope', ...
    'Logged actual force, force timestamps and direct acceleration; no model predictions','cases',[]);
for k=1:numel(d.inertiaContexts)
    c=d.inertiaContexts(k); file=fullfile(replayRoot,c.taskId,'result.mat');
    assert(strcmp(abfdFileHash(file),c.sourceHash),'ABFD:RawObservationSource','Frozen replay changed.');
    s=load(file,'result'); log=s.result.log;
    assert(strcmp(log.forceSource,'measured-actuator-output'),'ABFD:RawObservationForce');
    row=struct('taskId',c.taskId,'sourceFile',char(java.io.File(file).getCanonicalPath()), ...
        'sourceHash',abfdFileHash(file),'rows',numel(log.t),'x',[log.q;log.qd].', ...
        'time',log.t(:),'qdd',log.qdd.','actualForce',log.actualForce.', ...
        'forceTime',log.forceTime(:),'forceSource',log.forceSource);
    if isempty(report.cases), report.cases=row; else, report.cases(k)=row; end
end
writeAbfdJson(outputFile,report);
end
