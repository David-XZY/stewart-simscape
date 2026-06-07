function probe = probeFatropOptions(outputFile)
% probeFatropOptions - 逐项测试当前 CasADi FATROP MATLAB 接口接受的选项
if nargin < 1
    outputFile = '';
end

probe = struct();
probe.fatropPluginAvailable = false;
probe.fatropToySolvePassed = false;
probe.hessianApproximationAccepted = false;
probe.maxIterAccepted = false;
probe.manualStructureAccepted = false;
probe.nxNuNgAccepted = false;
probe.equalityMaskAccepted = false;
probe.lbfgsMemoryAccepted = false;
probe.failureReason = "";

try
    import casadi.*
    probe.fatropPluginAvailable = casadi.has_nlpsol('fatrop');
catch ME
    probe.failureReason = string(compactMessage(ME.message));
    writeProbe(outputFile, probe);
    return;
end
if ~probe.fatropPluginAvailable
    probe.failureReason = "CasADi 未发现 fatrop nlpsol 插件";
    writeProbe(outputFile, probe);
    return;
end

try
    import casadi.*
    x0 = MX.sym('x0'); u0 = MX.sym('u0'); x1 = MX.sym('x1');
    z = [x0; u0; x1];
    nlp = struct('x', z, 'f', sumsqr(z), 'g', x1 - x0 - u0);
    base = struct();
    base.structure_detection = 'manual';
    base.N = 1;
    base.nx = {1, 1};
    base.nu = {1, 0};
    base.ng = {0, 0};
    base.equality = {true};
    base.print_time = false;

    solver = nlpsol('fatrop_option_probe_toy', 'fatrop', nlp, base);
    sol = solver('x0', zeros(3, 1), 'lbx', -10*ones(3, 1), 'ubx', 10*ones(3, 1), ...
        'lbg', 0, 'ubg', 0); %#ok<NASGU>
    stats = solver.stats();
    probe.fatropToySolvePassed = isfield(stats, 'success') && stats.success;
    probe.manualStructureAccepted = true;
    probe.nxNuNgAccepted = true;
    probe.equalityMaskAccepted = true;

    probe.maxIterAccepted = optionAccepted(nlp, base, 'max_iter', 30);
    probe.lbfgsMemoryAccepted = optionAccepted(nlp, base, 'lbfgs_memory', 10);
    probe.hessianApproximationAccepted = optionAccepted(nlp, base, ...
        'hessian_approximation', 'limited-memory');
catch ME
    probe.failureReason = string(compactMessage(ME.message));
end

writeProbe(outputFile, probe);
end

function accepted = optionAccepted(nlp, base, name, value)
opts = base;
opts.fatrop = struct();
opts.fatrop.(name) = value;
try
    solverName = ['fatrop_probe_', regexprep(name, '[^A-Za-z0-9_]', '_')];
    solver = casadi.nlpsol(solverName, 'fatrop', nlp, opts);
    solver('x0', zeros(3, 1), 'lbx', -10*ones(3, 1), 'ubx', 10*ones(3, 1), ...
        'lbg', 0, 'ubg', 0);
    accepted = true;
catch
    accepted = false;
end
end

function writeProbe(outputFile, probe)
if isempty(outputFile)
    return;
end
fid = fopen(outputFile, 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
names = fieldnames(probe);
for i = 1:numel(names)
    value = probe.(names{i});
    if islogical(value) || isnumeric(value)
        fprintf(fid, '%s=%g\n', names{i}, value);
    else
        fprintf(fid, '%s=%s\n', names{i}, string(value));
    end
end
end

function message = compactMessage(message)
message = regexprep(char(string(message)), '\s+', ' ');
if numel(message) > 800
    message = [message(1:800), ' ...'];
end
end
