function identity=ch5SourceIdentity()
[c,root]=ch5Init();files=["ch5Physics.m","ch5AbsoluteState.m","ch5Rotation.m", ...
    "ch5CollisionModel.m","ch5SafetyQp.m","ch5Ihsid.m","ch5PlanPostcheck.m","ch5Reference.m", ...
    "ch5DiscreteLqi.m","ch5PrepareSimscape.m","ch5SimscapeSfun.m","ch5PlantModel.m","ch5Observe.m"];
digest=java.security.MessageDigest.getInstance('SHA-256');individual=struct();
keys={'version','Ts','internal_step','H','episode_seconds','replan_seconds','max_obstacles','safe_distance','sample_hold_margin','lqi_control_scale'};
baselineConfig=struct();for j=1:numel(keys),baselineConfig.(keys{j})=c.(keys{j});end
digest.update(unicode2native(jsonencode(baselineConfig),'UTF-8'));
for name=files
    file=fullfile(root,'chapter5',name);fid=fopen(file,'rb');bytes=fread(fid,inf,'*uint8');fclose(fid);
    one=java.security.MessageDigest.getInstance('SHA-256');one.update(bytes);
    value=lower(reshape(dec2hex(typecast(one.digest(),'uint8'),2).',1,[]));
    individual.(matlab.lang.makeValidName(name))=value;digest.update(bytes);
end
identity=struct('baseline_source_sha256',lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[])), ...
    'files',individual,'baseline_commit','ae06b7508151aea2c704d4b1a204bd9fb45e3347');
end
