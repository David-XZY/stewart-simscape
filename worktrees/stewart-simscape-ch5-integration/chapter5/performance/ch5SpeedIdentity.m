function identity=ch5SpeedIdentity()
directory=fileparts(mfilename('fullpath'));files=dir(fullfile(directory,'*.m'));identity=struct();
for k=1:numel(files)
    fid=fopen(fullfile(directory,files(k).name),'rb');bytes=fread(fid,inf,'*uint8');fclose(fid);
    hash=java.security.MessageDigest.getInstance('SHA-256');hash.update(bytes);
    identity.(matlab.lang.makeValidName(files(k).name))=lower(reshape(dec2hex(typecast(hash.digest(),'uint8'),2).',1,[]));
end
end
