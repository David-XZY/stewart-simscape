function value = abfdHash(data)
% Hash serializable experiment inputs (not function handles).
digest = java.security.MessageDigest.getInstance('SHA-256');
digest.update(unicode2native(jsonencode(data),'UTF-8'));
value = lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
