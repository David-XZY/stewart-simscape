function c3Prepare(runName)
c=c3Setup(runName); assert(isfile(fullfile(c.output,'standard.mat')));
checkNetworkExport(c.modelFile);
tests=jsondecode(fileread(fullfile(c.output,'core_tests.json'))); assert(tests.passed);
pilot=jsondecode(fileread(fullfile(c.output,'pilot.json'))); assert(pilot.complete);
cacheTest=jsondecode(fileread(fullfile(c.output,'cache_tests.json'))); assert(cacheTest.passed);
manifest=c3Manifest(c); jobs=c3Jobs(manifest,c); z=load(fullfile(c.output,'standard.mat'));
frozen=struct('implementation',c3Fingerprint(c),'manifest',manifest.hash,'standard',z.standard.hash, ...
    'model',dpHash(fileread(c.modelFile)),'dataset',dpHash(fileread(c.datasetFile)), ...
    'jobs',dpHash(jobs),'jobCount',numel(jobs),'matlab',version,'workers',1,'threads',1);
file=fullfile(c.output,'frozen.json');
if isfile(file), old=jsondecode(fileread(file)); assert(strcmp(dpHash(old),dpHash(frozen)),'C3:Frozen','New run required after a change.');
else, dpWriteJson(file,frozen); end
fprintf('C3_PREPARED %d conditions %d calls\n',numel(manifest.cases),numel(jobs));
end
