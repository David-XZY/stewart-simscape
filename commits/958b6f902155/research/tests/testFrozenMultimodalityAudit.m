function report=testFrozenMultimodalityAudit()
% Count zero-success frozen tasks in the denominator; no model is trained.
root=setupAbfdResearch(); directory=fullfile(root,'results','abfd','audit_contract_fixture');
file=fullfile(directory,'dataset.json'); output=fullfile(directory,'audit.json');
frozen=struct('taskId',{},'split',{});
for k=1:12, frozen(k)=struct('taskId',sprintf('task_%d',k),'split','train'); end
data=struct('status','test-fixture','groups',{{}},'routeMode',{{}},'split',{{}},'frozenTasks',frozen);
for k=1:4
    data.groups=[data.groups,repmat({frozen(k).taskId},1,4)];
    data.routeMode=[data.routeMode,{'left','left','right','right'}];
    data.split=[data.split,repmat({'train'},1,4)];
    if k==3
        writeAbfdJson(file,data); audit=run_F0_multimodality_audit(file,output);
        assert(audit.trainingTaskCount==12 && audit.trainingTasksWithFeasibleRows==3);
        assert(audit.multimodalTaskFraction==.25 && ~audit.diffusionApproved);
    end
end
writeAbfdJson(file,data); audit=run_F0_multimodality_audit(file,output);
assert(audit.diffusionApproved && abs(audit.multimodalTaskFraction-1/3)<1e-12);
report=struct('passed',true,'scope','zero-success tasks remain in frozen train denominator; fixture only');
end
