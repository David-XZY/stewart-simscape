function ch5SpeedWorker()
% Reuse MATLAB, physics and initialized models for successive diagnostics.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','performance_probe');
commandFile=fullfile(directory,'worker_command.json');last='';
fprintf('Speed worker ready PID %d\n',feature('getpid'));
while true
    if isfile(commandFile)
        command=jsondecode(fileread(commandFile));
        if ~strcmp(command.id,last)
            last=command.id;if strcmp(command.action,'stop'),break;end
            status=struct('id',last,'pid',feature('getpid'),'state','running');writeStatus(directory,status);
            try
                % Only explicitly queued development functions can execute.
                assert(startsWith(command.function,'ch5Speed'),'ch5:InvalidJob','Unsupported diagnostic job.');
                assert(~isempty(regexp(command.function,'^ch5Speed[A-Za-z0-9_]+$','once')),'ch5:InvalidJob','Invalid diagnostic function name.');
                clear ch5SpeedDiffusionControl ch5SpeedDispatch ch5SpeedPrepare
                eval(['clear ',command.function]);
                feval(command.function);
                status.state='complete';
            catch ex
                status.state='failed';status.message=getReport(ex,'extended','hyperlinks','off');fprintf('%s\n',status.message);
            end
            writeStatus(directory,status);
        end
    end
    pause(.5);
end
end
function writeStatus(directory,status)
fid=fopen(fullfile(directory,'worker_status.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(status));fclose(fid);
end
