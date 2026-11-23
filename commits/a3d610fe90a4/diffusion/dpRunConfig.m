function config=dpRunConfig(c)
% Scientific settings and numerical runtime; machine-local paths are excluded.
config=rmfield(c,{'root','output','dependencyRoot'});
config.matlabVersion=version; config.platform=computer;
config.casadiVersion=casadi.CasadiMeta.version();
end
