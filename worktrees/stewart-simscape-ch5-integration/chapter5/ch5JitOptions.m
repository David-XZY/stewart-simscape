function options=ch5JitOptions()
% Native compilation preserves the same mathematical functions/derivatives.
persistent serial
if isempty(serial),serial=0;end;serial=serial+1;
compilerDirectory='C:/msys64/ucrt64/bin';assert(isfile([compilerDirectory,'/gcc.exe']),'ch5:CompilerMissing','The pinned native compiler is required.');
if ~startsWith(getenv('PATH'),compilerDirectory),setenv('PATH',[compilerDirectory,pathsep,getenv('PATH')]);end
directory=fileparts(mfilename('fullpath'));python=fullfile(directory,'.venv','Scripts','python.exe');wrapper=fullfile(directory,'compile.py');
assert(~contains(python,' ')&&~contains(wrapper,' '),'ch5:CompilerPath','Compiler bridge paths must not contain spaces.');
command=sprintf('%s %s',python,wrapper);
c=jsondecode(fileread(fullfile(directory,'config.json')));
if c.native_mixed_optimization,command=[command,' --ch5-mixed-opt'];end
nativeDirectory=strrep(fullfile(directory,'artifacts','native',sprintf('process_%d',feature('getpid'))),'\','/');
if ~isfolder(nativeDirectory),mkdir(nativeDirectory);end
unique=sprintf('ch5_%d_%d',feature('getpid'),serial);
options=struct('jit',true,'jit_name',['jit_',unique],'jit_temp_suffix',false,'compiler','shell', ...
    'jit_options',struct('compiler',command,'flags',{{'-O0'}},'name',['obj_',unique], ...
    'directory',[nativeDirectory,'/'],'temp_suffix',false, ...
    'compiler_setup',' -c ','compiler_output_flag',' -o ','linker','gcc','linker_setup',' -shared ','linker_output_flag',' -o '));
end
