function hash=c3Fingerprint(c)
entries={};
for folder={'chapter3','study','diffusion','opt_minimal/core','src'}
    files=dir(fullfile(c.root,folder{1},'*.m'));
    for k=1:numel(files)
        entries{end+1}=struct('file',[folder{1},'/',files(k).name],'hash',dpHash(fileread(fullfile(files(k).folder,files(k).name)))); %#ok<AGROW>
    end
end
hash=dpHash(entries);
end
