addpath('D:/NPU/Matlab_code/stewart-simscape-diffusion/diffusion'); c=setupDiffusion();
files=dir(fullfile(c.root,'results','projection_source','attempts','*','*.mat'));
rows=struct('taskId',{},'start',{},'passed',{},'seconds',{},'iterations',{},'minClearance',{});
for fi=1:numel(files)
 s=load(fullfile(files(fi).folder,files(fi).name)); task=s.task; timer=tic;
 t=linspace(0,task.T,301); tr=dpReconstruct(s.result.bundle,t); [B,~,~]=dpBasis(t,task.T);
 base=dpBaseline(task); cp=base; cp(:,4:13)=0; cp(:,4:13)=(tr.q-cp*B)/B(4:13,:);
 report=dpValidate(struct('kind','spline','cp',cp,'T',task.T),task,c); iter=0;
 if ~report.passed
  asset=struct('outputMean',reshape(cp(:,4:13)-base(:,4:13),[],1),'outputScale',repmat([.1;.1;.1;.2;.2;.2],10,1));
  cc=c; cc.guidanceWeights=[10,1,1,0,0]; fn=dpGuidance(task,asset,cc); z=zeros(60,1);
  for iter=1:40
   [cost,grad]=fn(z); cost=full(cost); grad=full(grad); direction=grad/max(1e-12,sqrt(mean(grad.^2))); accepted=false;
   for backtrack=0:8
    trial=z-.02/(2^backtrack)*direction; value=full(fn(trial));
    if isfinite(value) && value<cost, z=trial; accepted=true; break; end
   end
   if mod(iter,5)==0 || ~accepted
    cp=base; cp(:,4:13)=base(:,4:13)+reshape(asset.outputMean+asset.outputScale.*z,6,10);
    report=dpValidate(struct('kind','spline','cp',cp,'T',task.T),task,c);
    if report.passed || ~accepted, break; end
   end
  end
 end
 rows(end+1)=struct('taskId',task.taskId,'start',s.record.start,'passed',report.passed,'seconds',toc(timer),'iterations',iter,'minClearance',report.minClearance);
 fprintf('%s start %d projected %d in %.2fs\n',task.taskId,s.record.start,report.passed,rows(end).seconds);
 dpWriteJson(fullfile(c.output,'projection_batch_probe.json'),rows);
end
