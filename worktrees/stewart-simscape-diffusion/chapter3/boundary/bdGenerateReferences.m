function records=bdGenerateReferences(task,seedIds,c)
% Labels never inferred from solver status alone; raw and fitted gates separate.
folder=fullfile(c.output,'references',task.taskId); if ~isfolder(folder), mkdir(folder); end
records={};
for seedId=seedIds
    file=fullfile(folder,sprintf('seed_%02d.mat',seedId)); jsonfile=strrep(file,'.mat','.json');
    if isfile(jsonfile), old=jsondecode(fileread(jsonfile)); assert(isfield(old,'codeHash')&&strcmp(old.codeHash,c.codeHash)&&old.controlCount==c.controlCount,'BD:VersionChanged','Use a new data version after code/representation changes.'); records{end+1}=old; continue; end
    record=struct('taskId',task.taskId,'groupId',task.groupId,'split',task.split,'seedId',seedId,'passed',false, ...
        'codeHash',c.codeHash,'controlCount',c.controlCount,'fitPassed',false,'reason','boundary_rejected','file',file,'taskHash',task.hash,'labelId',[task.taskId,sprintf('_s%d',seedId)]);
    result=[]; fitted=[]; c.clockHandle=tic; c.wallBudget=c.referenceBudget;
    try
        if task.endpointPassed
            result=bdSolve(task,bdSeed(task,seedId,c),c); record.passed=result.passed; record.reason=result.solverStatus;
            if result.passed
                s=linspace(0,task.T,401); tr=bdReconstruct(result.bundle,s); [B,~,~]=bdBasis(s,task.T,c.controlCount);
                cp=bdBaseline(task,c.controlCount); cp(:,4:end-3)=0; cp(:,4:end-3)=(tr.q-cp*B)/B(4:end-3,:);
                fitted=struct('kind','spline','cp',cp,'T',task.T);
                fitReport=bdValidate(fitted,task,c,tic,60); record.fitPassed=fitReport.passed; record.fitReason=fitReport.reason;
                record.rawQuality=bdQuality(result.bundle,task,c); record.fitQuality=bdQuality(fitted,task,c);
                record.context=task.context(:).'; delta=cp-bdBaseline(task,c.controlCount); record.y=reshape(delta(:,4:end-3),1,[]);
                record.features=features(result.bundle,task);
            end
        end
    catch e, record.reason=[e.identifier,': ',e.message]; end
    record.seconds=toc(c.clockHandle); save(file,'task','result','fitted','record','-v7'); dpWriteJson(jsonfile,record); records{end+1}=record;
end
end
function z=features(bundle,task)
tr=bdReconstruct(bundle,linspace(0,task.T,33)); center=zeros(3,33); axis=center;
for k=1:33
    R=rpy2rotmZYX(tr.q(4:6,k)); center(:,k)=tr.q(1:3,k)+R*task.scene.objectCylinder.center_P; axis(:,k)=R*task.scene.objectCylinder.axis_P;
end
% Physical scales fixed; cylinder axial spin is absent from this geometry feature.
z=reshape([center/.1;axis/.2],1,[]);
end
