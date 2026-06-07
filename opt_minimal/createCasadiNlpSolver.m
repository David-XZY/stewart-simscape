function [solver, opts, solverBackend, fatropStructure] = createCasadiNlpSolver(nlp, solverOptions)
% createCasadiNlpSolver - 按后端创建 CasADi NLP solver
%
% IPOPT 分支保持原有 MA27/limited-memory 设置；FATROP 支持 auto 诊断和
% manual 阶段结构。manual 的 int/bool vector 在 MATLAB 接口中必须用 cell 传入。
if nargin < 2 || isempty(solverOptions)
    solverOptions = struct();
end
import casadi.*

solverBackend = 'ipopt';
if isfield(solverOptions, 'solverBackend') && ~isempty(solverOptions.solverBackend)
    solverBackend = lower(char(string(solverOptions.solverBackend)));
end

fatropStructure = '';
switch solverBackend
    case 'ipopt'
        opts = makeCommonIpoptOptions(solverOptions);
        solver = nlpsol('solver', 'ipopt', nlp, opts);
    case 'fatrop'
        fatropStructure = 'auto';
        if isfield(solverOptions, 'fatropStructure') && ~isempty(solverOptions.fatropStructure)
            fatropStructure = char(string(solverOptions.fatropStructure));
        end
        opts = struct();
        opts.print_time = true;
        opts.structure_detection = fatropStructure;
        opts.fatrop = struct();
        opts.fatrop.max_iter = 300;
        if isfield(solverOptions, 'maxIter')
            opts.fatrop.max_iter = solverOptions.maxIter;
        end
        if isfield(solverOptions, 'debug')
            opts.debug = solverOptions.debug;
        end
        if strcmp(fatropStructure, 'manual')
            requiredFields = {'N', 'nx', 'nu', 'ng', 'equality'};
            for i = 1:numel(requiredFields)
                if ~isfield(solverOptions, requiredFields{i})
                    error('createCasadiNlpSolver:MissingFatropManualOption', ...
                        'FATROP manual 缺少选项 %s。', requiredFields{i});
                end
            end
            opts.N = solverOptions.N;
            opts.nx = numericVectorToCell(solverOptions.nx);
            opts.nu = numericVectorToCell(solverOptions.nu);
            opts.ng = numericVectorToCell(solverOptions.ng);
            opts.equality = logicalVectorToCell(solverOptions.equality);
        else
            if ~isfield(solverOptions, 'numEq') || ~isfield(solverOptions, 'numIneq')
                error('createCasadiNlpSolver:MissingFatropStructureSizes', ...
                    'FATROP auto 需要传入 numEq 和 numIneq 以设置 equality 掩码。');
            end
            opts.equality = [true(solverOptions.numEq, 1); false(solverOptions.numIneq, 1)];
        end
        solver = nlpsol('solver', 'fatrop', nlp, opts);
    otherwise
        error('createCasadiNlpSolver:UnknownBackend', '未知 solverBackend: %s。', solverBackend);
end
end

function c = numericVectorToCell(v)
v = double(v(:)).';
c = num2cell(v);
end

function c = logicalVectorToCell(v)
v = logical(v(:)).';
c = num2cell(v);
end
