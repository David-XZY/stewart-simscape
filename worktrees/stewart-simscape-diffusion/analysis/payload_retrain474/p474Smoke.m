function p474Smoke(dataset,output)
c=pgConfig('payload_retrain474_v1');c.returnPolicy='physical';d=jsondecode(fileread(dataset));rows=d.rows;if ~iscell(rows),rows=num2cell(rows);end
i=find(cellfun(@(r)strcmp(r.split,'train'),rows),1);row=rows{i};task=row.task;task=pgFinalizeTask(task,task.payload,c);
cp=bdBaseline(task,16);cp(:,4:end-3)=reshape(row.controls,6,10);b=struct('kind','spline','T',task.T,'cp',cp);
r=pgValidate(b,task,c);assert(r.passed);
model=struct('method','regression','contextSchema','pg_basic99_v1','controlCount',16,'outputRepresentation','absolute_free_controls_column_major', ...
    'inputMean',zeros(99,1),'inputScale',ones(99,1),'outputMean',row.controls(:),'outputScale',ones(60,1), ...
    'layers',struct('W',zeros(60,99),'b',zeros(60,1)));
asset=struct('model',model);
[result,state]=p474Plan(task,'regression',10,20261001,asset,c,[]);
assert(result.passed&&strcmp(result.returnStage,'direct'));assert(isempty(state.warm));
[late,~]=p474Plan(task,'regression',0,20261001,asset,c,[]);assert(~late.passed);
bad=b;bad.cp(:,4:end-3)=100;v=pgValidate(bad,task,c);assert(~v.passed);
% Reject a solver flag as evidence: the physical gate uses only trajectory.
fakeSolver=struct('success',false,'status','Maximum_Iterations_Exceeded'); %#ok<NASGU>
safe=pgValidate(b,task,c);assert(safe.passed);
metrics=p474Metrics(b,task,c);assert(metrics.lengthViolation<1e-6&&metrics.forceViolation<1e-6);
dpWriteJson(output,struct('passed',true,'safeDirectAccepted',result.passed,'lateRejected',~late.passed,'violatingRejected',~v.passed,'physicalAcceptanceIndependentOfSolverStatus',safe.passed,'noFallback',isempty(state.warm)));
end
