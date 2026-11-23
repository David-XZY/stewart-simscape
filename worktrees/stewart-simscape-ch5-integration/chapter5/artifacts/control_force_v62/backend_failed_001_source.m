function [U,z,info]=ch5ForceSampleV66(context,previous,w,noise)
% One persistent Python/PyTorch CPU session, one bridge call per batch.
% Exported MATLAB inference remains the independent parity reference.
assert(isfield(w,'checkpoint_file')&&strcmp(w.architecture,'temporal_cnn_v64'),'ch5:CpuBackend','Require explicit frozen temporal checkpoint.');
persistent predictor
if isempty(predictor)
    [~,root]=ch5Init();python=fullfile(root,'chapter5','.venv','Scripts','python.exe');environment=pyenv;
    if environment.Status=="NotLoaded"
        pyenv('Version',python,'ExecutionMode','OutOfProcess');
    else
        assert(strcmpi(environment.Executable,python)&&environment.ExecutionMode=="OutOfProcess",'ch5:PythonSession','Existing Python session has another interpreter or execution mode.');
    end
    path=fullfile(root,'chapter5','reduced','force_v62');
    if int64(py.sys.path.count(path))==0,py.sys.path.insert(int32(0),path);end
    predictor=py.importlib.import_module('predictor_v66');
end
timer=tic;result=predictor.predict(py.numpy.array(context(:).'),py.numpy.array(previous(:).'),py.numpy.array(noise(:).'),py.str(w.checkpoint_file));
U=reshape(double(result{1}),6,60,[]);z=reshape(double(result{2}),360,[]);
info=struct('seconds',toc(timer),'kernel_seconds',double(result{3}),'finite',all(isfinite(U),'all'), ...
    'parameterization','v_force_residual','architecture','temporal_cnn_v64','backend','persistent_cpu_pytorch', ...
    'terminal_alpha',double(w.alpha_bar(end)),'steps',20,'candidate_count',size(U,3));
end
