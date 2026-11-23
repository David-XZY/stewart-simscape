function pgWriteCodeIdentity(path)
c=pgConfig('payload_continuation_v2'); dpWriteJson(path,struct('matlabCodeHash',c.codeHash,'version',c.version));
end
