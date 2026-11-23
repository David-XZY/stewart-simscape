function h=dpHash(value)
if ischar(value), bytes=uint8(unicode2native(value,'UTF-8')); else, bytes=uint8(unicode2native(jsonencode(value),'UTF-8')); end
d=java.security.MessageDigest.getInstance('SHA-256'); d.update(bytes);
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
