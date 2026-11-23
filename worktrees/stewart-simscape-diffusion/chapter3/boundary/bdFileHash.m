function hash=bdFileHash(file)
fid=fopen(file,'rb'); assert(fid>=0,'BD:File','Missing artifact: %s',file);
cleanup=onCleanup(@()fclose(fid)); d=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid), bytes=fread(fid,1024*1024,'*uint8'); d.update(bytes); end
hash=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
