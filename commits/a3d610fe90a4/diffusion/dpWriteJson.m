function dpWriteJson(file,value)
folder=fileparts(file); if ~isfolder(folder), mkdir(folder); end
tmp=[file,'.tmp']; fid=fopen(tmp,'w','n','UTF-8'); assert(fid>=0,'DP:IO','Cannot open output.');
cleanup=onCleanup(@()fclose(fid)); fprintf(fid,'%s',jsonencode(value)); clear cleanup;
[ok,msg]=movefile(tmp,file,'f'); assert(ok,'DP:IO',msg);
end
