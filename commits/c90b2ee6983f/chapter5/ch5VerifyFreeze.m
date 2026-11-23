function ch5VerifyFreeze()
[~,root]=ch5Init();python=fullfile(root,'chapter5','.venv','Scripts','python.exe');
script=fullfile(root,'chapter5','freeze.py');
[status,message]=system(sprintf('"%s" "%s" --verify',python,script));
assert(status==0,'ch5:FreezeChanged','Frozen artifact mismatch: %s',message);
end
