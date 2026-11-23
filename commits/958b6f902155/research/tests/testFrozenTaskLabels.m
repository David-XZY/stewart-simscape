function report=testFrozenTaskLabels()
task=struct('taskId','task_1','split','test','model',struct('name','model','mass',2), ...
    'scene',struct('name','scene','q0',zeros(6,1)));
saved=task; saved.model.name='old bytes'; saved.scene.name='old scene bytes';
bundle=struct('model',saved.model,'scene',saved.scene);
[fixed,audit]=restoreAbfdFrozenTaskLabels(bundle,saved,task);
assert(isequaln(fixed.model,task.model) && isequaln(fixed.scene,task.scene) && numel(audit.restoredFields)==2);
bad=saved; bad.model.mass=bad.model.mass+eps(bad.model.mass);
badBundle=bundle; badBundle.model=bad.model;
numeric=rejects(@()restoreAbfdFrozenTaskLabels(badBundle,bad,task),'ABFD:HoldoutNumericContext');
bad=saved; bad.taskId='task_2';
identity=rejects(@()restoreAbfdFrozenTaskLabels(bundle,bad,task),'ABFD:HoldoutNumericContext');
badBundle=bundle; badBundle.scene.q0(1)=1;
reference=rejects(@()restoreAbfdFrozenTaskLabels(badBundle,saved,task),'ABFD:HoldoutReferenceContext');
assert(numeric && identity && reference,'ABFD:FrozenLabelsTest','A non-label change was accepted.');
report=struct('passed',true,'oneUlpParameterChangeRejected',numeric,'identityChangeRejected',identity, ...
    'referenceChangeRejected',reference,'scope','Exact numeric and identity guards; only two display labels can be restored');
end
function yes=rejects(action,id)
yes=false;
try
    action();
catch e
    yes=strcmp(e.identifier,id);
end
end
