function report=checkNetworkExport(modelFile)
a=jsondecode(fileread(modelFile)); probe=jsondecode(fileread(fullfile(fileparts(modelFile),'network_probe.json')));
y=probe.input(:); layers=a.layers; if iscell(layers), layers=[layers{:}]; end
for k=1:numel(layers), y=layers(k).W*y+layers(k).b(:); if k<numel(layers), y=tanh(y); end; end
error=max(abs(y-probe.output(:))); assert(error<2e-5,'DP:Export','PyTorch/MATLAB network mismatch.');
report=struct('passed',true,'maxAbsoluteError',error,'assetHash',dpHash(fileread(modelFile)));
dpWriteJson(fullfile(fileparts(modelFile),'matlab_export_check.json'),report);
end
