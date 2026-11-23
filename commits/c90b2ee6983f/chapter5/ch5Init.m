function [c, root] = ch5Init()
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'chapter5'));
addpath(fullfile(root,'src'));
addpath(genpath(fullfile(root,'controller')));
addpath(genpath(fullfile(root,'opt_minimal','core')));
addpath(fullfile(root,'matlab'),fullfile(root,'matlab','simscape_subsystems'));
addpath(fullfile(root,'opt_minimal','validation'));
addpath(fullfile(root,'lib','casadi-windows-matlabR2018b-v3.7.2'));
hsl=fullfile(root,'lib','CoinHSL-archive.v2023.11.17.x86_64-w64-mingw32-libgfortran5','bin');
if ~contains(getenv('PATH'),hsl),setenv('PATH',[hsl,pathsep,getenv('PATH')]);end
c=jsondecode(fileread(fullfile(root,'chapter5','config.json')));
for name=["artifacts","artifacts/validation","artifacts/tasks","artifacts/teacher", ...
        "artifacts/models","artifacts/development","artifacts/formal","artifacts/report"]
    directory=fullfile(root,'chapter5',name);if ~isfolder(directory),mkdir(directory);end
end
end
