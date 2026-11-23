function report=ch5SpeedRestoreV33AfterProbe()
% Restore the accepted development template after rejected controller trials.
[~,root]=ch5Init();r=ch5ReducedPrepareV48(4,"nonideal-force");r.c.episode_seconds=15;
r.reduced=struct('iterations',1,'backtracking',5,'linearization_collision_reserve',.0002,'velocity_weight',1, ...
    'factor_derivatives',false,'retain_numerical',false,'restoration_attempts',0,'reuse_sensitivities',true, ...
    'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);
r.algorithm_revision='reduced-v33';r.reduced_identity=ch5ReducedIdentity();r.speed.stop_on_unsafe=true;
ch5ReleaseEpisodeData(r.mode,'diffusion',r.task);assignin('base','ch5Runtime',r);
set_param([r.modelName,'/Strict CLF-CBF-QP Filter'],'FunctionName','ch5ReducedV33Sfun','Parameters','ch5Runtime');
set_param(r.modelName,'SimulationCommand','update');
report=struct('pid',feature('getpid'),'prediction_intervals',r.c.H,'control_step_seconds',r.c.Ts, ...
    'controller',get_param([r.modelName,'/Strict CLF-CBF-QP Filter'],'FunctionName'), ...
    'scope','Restored persistent model template; previously saved failed trials are unchanged');
assert(strcmp(report.controller,'ch5ReducedV33Sfun')&&r.c.H==60,'ch5:RestoreFailure','Accepted template restoration failed.');
file=fullfile(root,'chapter5','artifacts','reduced_probe','restored_v33_after_literature.json');
assert(~isfile(file),'ch5:ExistingRestore','Preserve the previous restoration record.');
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('Restored %s H%d on persistent PID%d\n',report.controller,r.c.H,report.pid);
end
