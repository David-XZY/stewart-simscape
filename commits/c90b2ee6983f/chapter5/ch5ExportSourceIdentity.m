function identity=ch5ExportSourceIdentity()
[~,root]=ch5Init();identity=ch5SourceIdentity();
file=fullfile(root,'chapter5','artifacts','runtime_source_identity.json');
fid=fopen([file,'.pending'],'w','n','UTF-8');assert(fid>=0);
fprintf(fid,'%s',jsonencode(identity));fclose(fid);movefile([file,'.pending'],file,'f');
end
