function value=readAbfdJsonUtf8(file)
% R2019a fileread otherwise decodes UTF-8 using the Windows system code page.
fid=fopen(file,'rb'); assert(fid>=0,'ABFD:JsonRead','Cannot read JSON file: %s',file);
cleanup=onCleanup(@()fclose(fid)); bytes=fread(fid,inf,'*uint8').';
if numel(bytes)>=3 && isequal(bytes(1:3),uint8([239,187,191])), bytes=bytes(4:end); end
value=jsondecode(native2unicode(bytes,'UTF-8')); clear cleanup;
end
