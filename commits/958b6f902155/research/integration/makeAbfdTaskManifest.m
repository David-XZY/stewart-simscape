function tasks=makeAbfdTaskManifest(kind,count,seed)
if nargin<2, count=60; end
if nargin<3
    seed=20260905;
    if strcmp(kind,'evaluation'), seed=seed+1000000; end
end
validatestring(kind,{'proposal','residual','evaluation'});
assert(count>=10,'ABFD:TaskCount','At least ten grouped tasks are required.');
cells=cell(1,count); nominal=buildOptModelCustom();
for k=1:count
    split='train'; if k>floor(.6*count), split='validation'; end
    if k>floor(.8*count), split='test'; end
    if strcmp(kind,'evaluation'), split='test'; end
    task=sampleTransferTask(seed+k,split); task.plantOverrides=struct();
    if strcmp(kind,'residual')
        task.plantOverrides=struct('payloadMassScale',task.model.dynamics.objectMass/nominal.dynamics.objectMass, ...
            'payloadComOffset',[.01*sin(k);.01*cos(k);.01*sin(2*k)]);
        task.model=nominal; task.context=encodeTaskContext(task);
    end
    cells{k}=task;
end
tasks=[cells{:}];
end
