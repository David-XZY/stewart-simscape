function h=sgImplementationHash(c)
files=[dir(fullfile(c.root,'study','*.m'));dir(fullfile(c.root,'study','run_study.py'))];
names={files.name}; [~,order]=sort(names); entries={};
for k=order
    if any(strcmp(files(k).name,{'sgExport.m','sgPlot.m'})), continue; end
    entries{end+1}=struct('name',files(k).name,'hash',dpHash(fileread(fullfile(files(k).folder,files(k).name)))); %#ok<AGROW>
end
h=dpHash(struct('study',{entries},'legacyDiffusion',dpImplementationHash(c)));
end
