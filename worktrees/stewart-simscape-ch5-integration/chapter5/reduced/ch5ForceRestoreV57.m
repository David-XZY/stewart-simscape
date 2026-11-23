function ch5ForceRestoreV57(names,exists,saved,model,block,oldFunction)
% Anonymous onCleanup callers capture values; nested-workspace destruction is avoided.
for ii=1:numel(names)
    if exists(ii),assignin('base',names{ii},saved{ii});else,evalin('base',['clear ',names{ii}]);end
end
if nargin>3&&bdIsLoaded(model),set_param(block,'FunctionName',oldFunction,'Parameters','ch5Runtime');end
end
