function identity=ch5ExecutionIdentity()
% Record preparation/driver changes separately from unchanged control math.
directory=fileparts(mfilename('fullpath'));files=["campaign.py","ch5RunStage.m","ch5RunEpisode.m", ...
    "ch5PersistentWorker.m","ch5PrepareCachedSimscape.m","ch5ReleaseEpisodeData.m","ch5ExecutionIdentity.m"];
identity=struct('revision','persistent-worker-v1','files',struct());digest=java.security.MessageDigest.getInstance('SHA-256');
for file=files
    fid=fopen(fullfile(directory,file),'rb');assert(fid>=0);bytes=fread(fid,inf,'*uint8');fclose(fid);
    one=java.security.MessageDigest.getInstance('SHA-256');one.update(bytes);
    identity.files.(matlab.lang.makeValidName(file))=lower(reshape(dec2hex(typecast(one.digest(),'uint8'),2).',1,[]));
    digest.update(unicode2native(file,'UTF-8'));digest.update(bytes);
end
identity.sha256=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
