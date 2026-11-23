function run_dp_worker(kind,worker,workers,runName)
c=setupDiffusion(); c.output=fullfile(c.output,runName); if ~isfolder(c.output), mkdir(c.output); end
manifestFile=fullfile(c.output,'manifest.mat');
switch kind
    case {'pilot','formal'}
        assert(worker==1,'DP:Manifest','Single writer must freeze manifest.');
        assert(~isfile(manifestFile),'DP:ManifestExists','Reuse existing frozen manifest.');
        manifest=dpManifest(kind,c); save(manifestFile,'manifest','-v7'); dpWriteJson(fullfile(c.output,'manifest.json'),manifest);
    case 'data'
        s=load(manifestFile); generateDataset(s.manifest,c,worker,workers);
    case 'export'
        s=load(manifestFile); dpExportDataset(s.manifest,c);
    case 'benchmark'
        s=load(manifestFile); modelFile=fullfile(c.output,'training','model.json'); runBenchmark(s.manifest,modelFile,c,worker,workers);
    case 'benchmark_merge'
        s=load(manifestFile); mergeBenchmark(s.manifest,c);
    otherwise, error('DP:Action','Unknown worker action.');
end
end
