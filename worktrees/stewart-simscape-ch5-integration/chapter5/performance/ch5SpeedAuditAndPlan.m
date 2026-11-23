function ch5SpeedAuditAndPlan()
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','performance_probe');
failed=load(fullfile(directory,'corrected_closed_loop','003_ideal_force.mat'),'output','records');
info=struct('task',3,'status','simulation_failed','record_count',numel(failed.records),'message',failed.output.ErrorMessage);
fid=fopen(fullfile(directory,'corrected_closed_loop','003_failure.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(info));fclose(fid);
fprintf('Task3 failure before first saved sample: %s\n',info.message);
clear ch5SpeedPlanningPaired
ch5SpeedPlanningPaired;
end
