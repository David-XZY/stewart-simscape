function bdBuildData(tasksFile,split,runName,maxAttempts,controlCount)
if nargin<5, controlCount=16; end
assert(any(controlCount==[16,20,24]),'BD:Representation','Unsupported development control count.');
c=bdConfig(runName); c.controlCount=controlCount; manifest=jsondecode(fileread(tasksFile)); entries=manifest.tasks;
if iscell(entries), entries=[entries{:}]; end
count=0; rows={};
for k=1:numel(entries)
    e=entries(k); if ~strcmp(e.split,split), continue; end
    task=bdMakeTask(e,e.parameters,c); seeds=1:min(4,maxAttempts-count);
    if isempty(seeds), break; end
    records=bdGenerateReferences(task,seeds,c); rows=[rows,records]; count=count+numel(seeds);
    if strcmp(split,'train') && count<maxAttempts
        valid=cellfun(@(r)r.passed&&r.fitPassed,records);
        if sum(valid)<2
            extra=5:min(8,4+maxAttempts-count); more=bdGenerateReferences(task,extra,c); rows=[rows,more]; count=count+numel(extra);
        end
    end
end
dpWriteJson(fullfile(c.output,[split,'_reference_index.json']),struct('records',{rows},'attempts',count));
end
