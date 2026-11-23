function report=ch5SpeedTeacherQpProbeV83()
% Identify uncontrollable initial-state tightening before changing any bounds.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_failure_v83');
assert(~isfile(fullfile(directory,'qp_probe.json')),'ch5:ExistingAttempt','Preserve QP diagnostic evidence.');
source=fullfile(fileparts(directory),'teacher_pilot_v82','train_016_nonideal_force.mat');
d=load(source,'records');r=ch5ReducedPrepareV48(4,'nonideal-force');
report=struct('completed',false,'formal',false,'pid',feature('getpid'),'rows',{{}});
for time=[13.72,13.74,13.76,13.78]
    [~,index]=min(abs(cellfun(@(z)z.time,d.records)-time));z=d.records{index};
    [~,trace]=r.plant.fd(z.x,z.u,zeros(6,1));trace=full(trace);values=zeros(25,size(trace,2));
    for j=1:size(trace,2),values(:,j)=full(r.plant.mechanicalValues(trace(:,j),z.u));end
    row=struct('time',z.time,'original_qp_feasible',z.diagnostic.original_qp_feasible, ...
        'final_feasible',z.diagnostic.feasible,'initial_acceleration',values(13:18,1), ...
        'prediction_limit',ch5PredictionAccelerationLimit(r),'physical_limit',r.physical.actuator.lddotMax, ...
        'initial_prediction_band_violation',max([abs(values(13:18,1))-ch5PredictionAccelerationLimit(r);0]), ...
        'interval_acceleration_maximum',max(abs(values(13:18,:)),[],2),'trace_time',r.plant.traceTime);
    % For a delayed actuator, the current force is part of x and cannot be
    % changed by the present command. Measure this exact zero derivative.
    import casadi.*
    x=MX.sym('probe_x',r.plant.n);u=MX.sym('probe_u',6);v=r.plant.mechanicalValues(x,u);
    fn=Function('probe_initial_acceleration',{x,u},{jacobian(v(13:18),u)});
    row.initial_acceleration_input_jacobian_max=max(abs(full(fn(z.x,z.u))),[],'all');
    report.rows{end+1}=row;
    fprintf('V83 QP probe %.2f initial acceleration%.6f reserve violation%.6f max J%.3g\n',time,max(abs(row.initial_acceleration)),row.initial_prediction_band_violation,row.initial_acceleration_input_jacobian_max);
end
report.completed=true;writeJSON(fullfile(directory,'qp_probe.json'),report);
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
