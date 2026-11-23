addpath('D:/NPU/Matlab_code/stewart-simscape-diffusion/diffusion'); c=setupDiffusion();
s=load(fullfile(c.output,'stage1','simple_01.mat')); task=s.task;
t=linspace(0,task.T,301); tr=dpReconstruct(s.result.bundle,t); [B,~,~]=dpBasis(t,task.T);
base=dpBaseline(task); cp=base; cp(:,4:13)=0; cp(:,4:13)=(tr.q-cp*B)/B(4:13,:);
asset=struct('outputMean',reshape(cp(:,4:13)-base(:,4:13),[],1),'outputScale',repmat([.1;.1;.1;.2;.2;.2],10,1));
cc=c; cc.guidanceWeights=[10,1,1,0,0]; timer=tic; fn=dpGuidance(task,asset,cc); z=zeros(60,1);
for iter=1:100
 [cost,grad]=fn(z); cost=full(cost); grad=full(grad); direction=grad/max(1e-12,sqrt(mean(grad.^2))); accepted=false;
 for backtrack=0:8
  trial=z-.02/(2^backtrack)*direction; value=full(fn(trial));
  if isfinite(value) && value<cost, z=trial; accepted=true; break; end
 end
 if mod(iter,5)==0 || ~accepted
  cp=base; cp(:,4:13)=base(:,4:13)+reshape(asset.outputMean+asset.outputScale.*z,6,10);
  report=dpValidate(struct('kind','spline','cp',cp,'T',task.T),task,c);
  fprintf('Projection %d pass=%d clearance %.8f cost %.4g elapsed %.3fs\n',iter,report.passed,report.minClearance,value,toc(timer));
  if report.passed || ~accepted, break; end
 end
end
save(fullfile(c.output,'projection_probe.mat'),'cp','report');
