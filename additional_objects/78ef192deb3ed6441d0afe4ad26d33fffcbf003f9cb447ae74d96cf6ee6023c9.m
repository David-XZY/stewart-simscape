function bdDiagnose(tasksFile,libraryFile,runName,maxPaths)
c=bdConfig(runName); asset=bdLoadLibrary(libraryFile,'train',c); manifest=jsondecode(fileread(tasksFile)); entries=manifest.tasks;
if iscell(entries), entries=[entries{:}]; end
count=0; names={'A0','A5','A7','A5-C'};
for k=1:numel(entries)
    e=entries(k); task=bdMakeTask(e,e.parameters,c); folder=fullfile(c.output,'diagnostics',task.taskId); if ~isfolder(folder), mkdir(folder); end
    states={}; expected=numel(asset.library)*4;
    for j=1:numel(asset.library)
        for r=1:4
            if count>=maxPaths, break; end
            file=fullfile(folder,sprintf('path_%05d_%s.json',j,names{r})); count=count+1;
            if isfile(file), d=jsondecode(fileread(file));
            else
                try
                    path=bdAdapt(asset.library{j},task,names{r},c); d=bdPathDiagnostic(path,task,c);
                catch err
                    d=struct('status','unknown','excluded',false,'reason',[err.identifier,': ',err.message],'lowerBound',[]);
                end
                d.labelId=asset.library{j}.labelId; d.adapterId=names{r}; dpWriteJson(file,d);
            end
            states{end+1}=d;
        end
        if count>=maxPaths, break; end
    end
    excluded=cellfun(@(x)x.excluded,states); active=states(~excluded); bounds=[]; missing=false;
    for j=1:numel(active)
        if isempty(active{j}.lowerBound), missing=true; else, bounds(end+1)=active{j}.lowerBound.timeLowerBound; end
    end
    complete=numel(states)==expected; minBound=[]; if ~isempty(bounds)&&~missing, minBound=min(bounds); end
    summary=struct('taskId',task.taskId,'libraryHash',asset.libraryHash,'complete',complete,'expected',expected,'processed',numel(states), ...
        'excluded',sum(excluded),'unknownBounds',missing,'minTimeLowerBound',minBound,'geometricReuseExcluded',complete&&all(excluded), ...
        'retimingExcludedByNumericBound',complete&&~isempty(active)&&~missing&&all(bounds>task.T+c.timeError), ...
        'newPathEvidenceRequired',true,'scope','numeric diagnostics; sampled separation is not continuous certification');
    dpWriteJson(fullfile(folder,'summary.json'),summary);
    if count>=maxPaths, break; end
end
end
