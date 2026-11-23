function manifest=exportExperimentManifest(outputDir,config,status,details)
if nargin<4, details=struct(); end
if ~isfolder(outputDir), mkdir(outputDir); end
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
[~,commit]=system(sprintf('git -C "%s" rev-parse HEAD',root));
[~,dirty]=system(sprintf('git -C "%s" status --porcelain',root));
manifest=struct('schemaVersion',1,'status',status,'config',config, ...
    'configHash',abfdHash(config),'commit',strtrim(commit),'dirty',~isempty(strtrim(dirty)), ...
    'createdUtc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ss''Z''')), ...
    'matlab',version,'details',details);
file=fullfile(outputDir,'manifest.json'); temp=[file,'.tmp'];
fid=fopen(temp,'w','n','UTF-8'); assert(fid>=0,'ABFD:WriteManifest','Cannot write manifest.');
cleanup=onCleanup(@()fclose(fid)); fprintf(fid,'%s',jsonencode(manifest,PrettyPrint=true));
clear cleanup; movefile(temp,file,'f');
end
