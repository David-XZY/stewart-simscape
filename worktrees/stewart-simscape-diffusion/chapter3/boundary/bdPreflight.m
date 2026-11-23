function report=bdPreflight()
basic=bdTests(); integration=bdIntegrationTests();
c=bdConfig('boundary_preflight_v1');
audit=fullfile(c.root,'results','chapter3','chapter3_v3','label_audit.json');
bdImportLegacy(audit,'boundary_legacy_v1');
task=bdMakeTask(struct('taskId','diagnostic_prototype','groupId','development'),[0,0,0,7.5],c);
tasksFile=fullfile(c.output,'diagnostic_tasks.json'); dpWriteJson(tasksFile,struct('tasks',{{task}}));
libraryFile=fullfile(c.root,'results','chapter3','boundary_legacy_v1','legacy_library.json');
bdDiagnose(tasksFile,libraryFile,'boundary_diagnostic_smoke_v1',8);
library=jsondecode(fileread(libraryFile));
report=struct('basic',basic,'integration',integration,'legacyTrainingLabels',numel(library.labels), ...
    'diagnosticPathsRequested',8,'codeHash',c.codeHash,'formalReady',false, ...
    'reason','Physical domain and mechanism boundary have not been frozen; no formal training or experiment is authorized by this preflight.');
dpWriteJson(fullfile(c.output,'preflight.json'),report);
fprintf('BD_PREFLIGHT_DONE labels=%d\n',numel(library.labels));
end
