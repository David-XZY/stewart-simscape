function pgCheckpointValidate(inputPath,outputPath)
input=jsondecode(fileread(inputPath)); c=pgConfig('payload_checkpoint_validation'); rows=cell(numel(input),1);
for k=1:numel(input)
    if iscell(input), x=input{k}; else, x=input(k); end
    rows{k}=struct('taskId',x.taskId,'passed',false,'reason','candidate_incomplete');
    if isempty(x.controls), continue; end
    task=pgFinalizeTask(x.task,x.task.payload,c); n=numel(x.controls)/6+6; c.controlCount=n;
    cp=bdBaseline(task,n); cp(:,4:end-3)=reshape(x.controls,6,n-6); b=struct('kind','spline','cp',cp,'T',task.T);
    r=pgValidate(b,task,c); rows{k}.passed=r.passed; rows{k}.reason=r.reason;
end
dpWriteJson(outputPath,rows);
end
