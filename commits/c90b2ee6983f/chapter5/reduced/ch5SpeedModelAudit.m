function report=ch5SpeedModelAudit()
% Compare actual consecutive Simscape samples with the admitted fd model.
[c,root]=ch5Init();[physical,scene,ad,strict]=ch5Physics(c);
directory=fullfile(root,'chapter5','artifacts','reduced_probe','model_audit');if ~isfolder(directory),mkdir(directory);end
files={ ...
    'performance_probe/tail_closed_loop/001_ideal_force.mat','ideal-force'; ...
    'performance_probe/corrected_closed_loop/001_ideal_force.mat','ideal-force'; ...
    'development/development_001_ideal_force_baseline_0.mat','ideal-force'; ...
    'teacher/train_001_ideal_force_baseline_0.mat','ideal-force'; ...
    'teacher/train_001_nonideal_force_baseline_0.mat','nonideal-force'};
report=struct('pid',feature('getpid'),'rows',{{}},'scope','Paired one-step dynamics audit; no new closed-loop conclusions');
for fi=1:size(files,1)
    mode=string(files{fi,2});profile=ch5ActuatorProfile(mode,c,physical);
    plant=ch5PlantModel(physical,scene,profile,struct('Ts',c.Ts,'substeps',ceil(c.Ts/c.internal_step),'Np',1,'integralGridTs',0));
    saved=load(fullfile(root,'chapter5','artifacts',files{fi,1}),'records','audit');records=saved.records;
    count=min(100,numel(records)-1);actual=zeros(12,count);predicted=actual;affineDifference=zeros(6,count);
    forceActual=[];if isfield(saved,'audit'),forceActual=saved.audit;end
    for j=1:count
        entry=records{j};next=records{j+1};
        preview=full(plant.fd(entry.x,entry.u,zeros(6,1)));predicted(:,j)=preview(1:12);
        actual(:,j)=[next.q;next.qd];
        [d,G]=ad.dynamics(entry.q,entry.qd);rhs=full(ad.motion(entry.q,entry.qd,entry.u,zeros(6,1)));
        affineDifference(:,j)=rhs(7:12)-full(d+G*entry.u);
    end
    % Samples may straddle a force switch. Compare the actual logged force
    % over each interval with the command recorded at its beginning.
    commandMismatch=zeros(6,count);
    if ~isempty(forceActual)
        for j=1:count
            mask=forceActual.time>=records{j}.time+1e-6 & forceActual.time<records{j+1}.time-1e-6;
            if any(mask),commandMismatch(:,j)=mean(forceActual.actual_force(mask,:),1)'-records{j}.u;end
        end
    end
    residual=actual-predicted;
    row=struct('file',files{fi,1},'mode',char(mode),'samples',count, ...
        'position_rms_m',sqrt(mean(sum(residual(1:3,:).^2,1))), ...
        'linear_velocity_rms_mps',sqrt(mean(sum(residual(7:9,:).^2,1))), ...
        'rotation_rms_rad',sqrt(mean(sum(residual(4:6,:).^2,1))), ...
        'angular_rate_rms_radps',sqrt(mean(sum(residual(10:12,:).^2,1))), ...
        'maximum_residual',max(abs(residual),[],2),'affine_model_maximum_difference',max(abs(affineDifference),[],2), ...
        'command_actual_force_rms',sqrt(mean(commandMismatch.^2,2)), ...
        'actual',actual,'predicted',predicted,'residual',residual);
    report.rows{end+1}=row;
    fprintf('Model audit %s: pos %.6g m, velocity %.6g m/s, angular rate %.6g rad/s, affine %.3g\n',files{fi,1},row.position_rms_m,row.linear_velocity_rms_mps,row.angular_rate_rms_radps,max(row.affine_model_maximum_difference));
    save(fullfile(directory,'audit.mat'),'report','-v7');
end
report.completed=true;save(fullfile(directory,'audit.mat'),'report','-v7');
fid=fopen(fullfile(directory,'audit.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
