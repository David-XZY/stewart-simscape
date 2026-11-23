function summary=pgDiagnoseLibrary(manifestPath,tasksPath,runName,maxPaths)
if nargin<4, maxPaths=512; end
c=pgConfig(runName); assets=pgAssets(manifestPath); specs=jsondecode(fileread(tasksPath)); specs=specs.tasks;
timer=tic; count=0; rows={}; names={'A0','A5','A7','A5-C'};
for ti=1:numel(specs)
    task=pgMakeTask(specs(ti),c);
    for hi=1:numel(assets.library)
        for ai=1:4
            if count>=maxPaths||toc(timer)>24*3600, break; end
            count=count+1; started=tic;
            row=struct('taskId',task.taskId,'sourceLabelId',assets.library{hi}.labelId,'adapterId',names{ai},'status','unknown','excluded',false,'timeLowerBound',[]);
            try
                path=bdAdapt(assets.library{hi},task,names{ai},c); diag=pgPathDiagnostic(path,task,c);
                row.status=diag.status; row.excluded=diag.excluded; row.diagnostic=diag;
                if ~diag.excluded
                    bounds=cell(1,3); for k=1:3, n=[257,513,1025]; bounds{k}=pgTimeLowerBound(path,task.model,linspace(0,1,n(k)),1e-8); end
                    row.bounds=bounds; row.timeLowerBound=bounds{end}.timeLowerBound;
                end
                row.pathHash=dpHash(path);
            catch e, row.failureReason=[e.identifier,':',e.message]; end
            row.seconds=toc(started); rows{end+1}=row; dpWriteJson(fullfile(c.output,sprintf('path_%07d.json',count)),row);
        end
        if count>=maxPaths||toc(timer)>24*3600, break; end
    end
    if count>=maxPaths||toc(timer)>24*3600, break; end
end
summary=struct('pathsProcessed',count,'pathsRequested',numel(specs)*numel(assets.library)*4,'seconds',toc(timer),'records',{rows}, ...
    'unprocessedStatus','unknown','scope','Offline diagnostic; finite failures do not exclude arbitrary retiming.');
dpWriteJson(fullfile(c.output,'diagnostic_summary.json'),summary);
end
