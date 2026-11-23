function writeAbfdJson(file,data)
folder=fileparts(file); if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
temp=[char(file),'.tmp']; fid=fopen(temp,'w','n','UTF-8');
assert(fid>=0,'ABFD:WriteFailed','Cannot write %s.',file);
cleanup=onCleanup(@()fclose(fid)); fprintf(fid,'%s',jsonencode(data)); clear cleanup;
movefile(temp,file,'f');
end
