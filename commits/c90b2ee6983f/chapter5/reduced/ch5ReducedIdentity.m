function identity=ch5ReducedIdentity()
directory=fileparts(mfilename('fullpath'));files=dir(fullfile(directory,'*.m'));identity=struct();
for j=1:numel(files)
    fid=fopen(fullfile(directory,files(j).name),'rb');bytes=fread(fid,inf,'*uint8');fclose(fid);
    hash=java.security.MessageDigest.getInstance('SHA-256');hash.update(bytes);
    identity.(matlab.lang.makeValidName(files(j).name))=lower(reshape(dec2hex(typecast(hash.digest(),'uint8'),2).',1,[]));
end
end
