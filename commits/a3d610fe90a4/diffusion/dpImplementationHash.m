function h=dpImplementationHash(c)
files=[dir(fullfile(c.root,'diffusion','*.m'));dir(fullfile(c.root,'diffusion','*.py'))];
[~,ix]=sort({files.name}); entries=cell(1,numel(files));
for k=1:numel(files), f=files(ix(k)); entries{k}=struct('name',f.name,'hash',dpHash(fileread(fullfile(f.folder,f.name)))); end
h=dpHash(entries);
end
