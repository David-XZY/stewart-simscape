function report=ch5SpeedForceInferenceV66()
% Same weights, explicit noise, full-DDIM comparison and bridge timings.
clear ch5ForceSampleV66
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62');
assert(~isfile(fullfile(directory,'inference_v66.json')),'ch5:ExistingAttempt','Preserve backend comparisons.');
w=load(fullfile(directory,'temporal_v64','seed_5101','weights.mat'));w.checkpoint_file=fullfile(directory,'temporal_v64','seed_5101','model.pt');
c=double(w.golden_context).*double(w.context_std(:))+double(w.context_mean(:));noise=double(w.golden_z);
[Ud,zd]=ch5ForceSampleV64(c,zeros(6,1),w,noise);timer=tic;[Up,zp]=ch5ForceSampleV66(c,zeros(6,1),w,noise);cold=toc(timer);
times=zeros(12,2);kernels=zeros(12,1);
for i=1:12
    if mod(i,2),order=[1,2];else,order=[2,1];end
    for j=order
        timer=tic;if j==1,ch5ForceSampleV64(c,zeros(6,1),w,noise);else,[~,~,info]=ch5ForceSampleV66(c,zeros(6,1),w,noise);kernels(i)=info.kernel_seconds;end
        times(i,j)=toc(timer);
    end
end
report=struct('pid',feature('getpid'),'completed',true,'formal',false,'cold_python_seconds',cold,'python_execution_mode','OutOfProcess', ...
    'sample_python_error',max(abs(zp-double(w.golden_sample)),[],'all'),'sample_double_error',max(abs(zp-zd),[],'all'), ...
    'force_double_error_N',max(abs(Up-Ud),[],'all'),'double_median_seconds',median(times(:,1)), ...
    'backend_median_seconds',median(times(:,2)),'backend_p95_seconds',prctile(times(:,2),95),'kernel_median_seconds',median(kernels),'times',times);
report.accepted=max([report.sample_python_error,report.sample_double_error])<1e-4;
save(fullfile(directory,'inference_v66.mat'),'report','-v7');fid=fopen(fullfile(directory,'inference_v66.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(report.accepted,'ch5:BackendParity','Persistent CPU backend exceeds full-DDIM parity tolerance.');
end
