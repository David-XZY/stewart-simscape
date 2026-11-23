function value=abfdFileHash(file)
if strlength(string(file))==0, value=''; return; end
assert(isfile(file),'ABFD:MissingArtifact','Missing artifact: %s',file);
fid=fopen(file,'rb'); cleanup=onCleanup(@()fclose(fid));
d=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid), d.update(fread(fid,1024*1024,'*uint8')); end
value=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
