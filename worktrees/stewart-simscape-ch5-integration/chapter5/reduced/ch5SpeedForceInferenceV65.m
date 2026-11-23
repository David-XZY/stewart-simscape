function report=ch5SpeedForceInferenceV65()
% Precision and complete-DDIM latency, identical frozen weights/noise.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62');
assert(~isfile(fullfile(directory,'inference_v65.json')),'ch5:ExistingAttempt','Preserve inference experiments.');
w=load(fullfile(directory,'temporal_v64','seed_5101','weights.mat'));
c=double(w.golden_context).*double(w.context_std(:))+double(w.context_mean(:));noise=double(w.golden_z);
[Ud,zd]=ch5ForceSampleV64(c,zeros(6,1),w,noise);[Us,zs]=ch5ForceSampleV65(c,zeros(6,1),w,noise);
v=ch5ForceDenoiseV65(noise,double(w.golden_context),double(w.golden_t),w);
times=zeros(12,2);
for i=1:12
    if mod(i,2),order=[1,2];else,order=[2,1];end
    for j=order
        timer=tic;if j==1,ch5ForceSampleV64(c,zeros(6,1),w,noise);else,ch5ForceSampleV65(c,zeros(6,1),w,noise);end
        times(i,j)=toc(timer);
    end
end
report=struct('pid',feature('getpid'),'completed',true,'formal',false,'samples_per_batch',4,'ddim_steps',20, ...
    'v_python_error',max(abs(double(v)-double(w.golden_v)),[],'all'), ...
    'sample_python_error',max(abs(zs-double(w.golden_sample)),[],'all'), ...
    'sample_double_error',max(abs(zs-zd),[],'all'),'force_double_error_N',max(abs(Us-Ud),[],'all'), ...
    'double_median_seconds',median(times(:,1)),'single_median_seconds',median(times(:,2)), ...
    'double_p95_seconds',prctile(times(:,1),95),'single_p95_seconds',prctile(times(:,2),95),'times',times);
report.accepted=max([report.v_python_error,report.sample_python_error,report.sample_double_error])<1e-4;
save(fullfile(directory,'inference_v65.mat'),'report','-v7');
fid=fopen(fullfile(directory,'inference_v65.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(report.accepted,'ch5:TemporalPrecision','Single precision full-DDIM inference differs above tolerance.');
end
