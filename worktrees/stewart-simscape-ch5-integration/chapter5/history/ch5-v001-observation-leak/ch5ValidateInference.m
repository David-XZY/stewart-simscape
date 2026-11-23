function report=ch5ValidateInference()
[c,root]=ch5Init();report=struct('version',c.version,'seeds',c.training_seeds,'errors',zeros(5,1));
for i=1:5
    file=fullfile(root,'chapter5','artifacts','models',sprintf('seed_%d.mat',c.training_seeds(i)));w=load(file);
    actual=ch5Denoise(double(w.example_action),double(w.example_context),double(w.example_t),w);
    report.errors(i)=max(abs(actual-double(w.example_expected)),[],'all');
    assert(report.errors(i)<1e-5,'ch5:InferenceMismatch','MATLAB/PyTorch inference mismatch.');
end
report.passed=true;save(fullfile(root,'chapter5','artifacts','validation','inference.mat'),'report');
end
