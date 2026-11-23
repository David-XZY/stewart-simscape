function ch5ReleaseEpisodeData(mode,method,task)
% Saved episode arrays can be large. No completed run stays in appdata.
for name=["ch5Records","ch5Predictions","ch5Plans","ch5Audit","ch5State","ch5Random","ch5DiffusionWarm"]
    if isappdata(0,name),rmappdata(0,name);end
end
evalin('base','clear ch5Runtime');
persistent previousMode previousMethod obstacleSignature
if nargin==0,return;end
mode=string(mode);method=string(method);
[~,observed]=ch5Observe(task,0);signature=mat2str(find(observed(22,:)>.5));
if ~isempty(previousMode)&&(mode~=previousMode||method~=previousMethod)
    clear ch5Ihsid ch5PredictiveOptimizer ch5NativePreview ch5NativeNodes
    obstacleSignature='';
end
% IHSID has one graph per active set; discard former task signatures. Its
% numeric boundary/obstacle parameters are still supplied anew each solve.
if method=="baseline"&&~isempty(obstacleSignature)&&~strcmp(signature,obstacleSignature)
    clear ch5Ihsid
end
previousMode=mode;previousMethod=method;obstacleSignature=signature;
end
