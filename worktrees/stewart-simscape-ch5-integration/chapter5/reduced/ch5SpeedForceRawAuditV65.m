function report=ch5SpeedForceRawAuditV65()
% Disaggregate raw rollout rejection without comparing unlike physical units.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62');
assert(~isfile(fullfile(directory,'raw_audit_v65.json')),'ch5:ExistingAttempt','Preserve raw rejection audits.');
old=fullfile(root,'chapter5','artifacts','control_force_v57','pilot');s=load(fullfile(old,'selection.mat'));
data={load(fullfile(old,'candidates_5101.mat')),load(fullfile(directory,'temporal_v64','candidates_5101.mat'))};
labels={'mlp_v60','temporal_v64'};
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'completed',false,'formal',false,'rows',{{}});
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);r.reduced=struct('factor_derivatives',true);k=ch5ForceKernelsV64(r);
    for model=1:2
        d=data{model};
        for i=1:size(d.residual,2)
            index=d.selection_index(i);c=double(s.context(:,index));if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
            U=reshape(double(d.residual(:,i)),6,60)+c(31:36);x=c(1:r.plant.n);
            [~,values,~]=k.preview(x,U);values=full(values);
            acceleration=abs(values(13:18,:));[peak,where]=max(acceleration,[],'all','linear');[leg,point]=ind2sub(size(acceleration),where);
            lengthViolation=max([r.physical.lmin-values(1:6,:);values(1:6,:)-r.physical.lmax;zeros(1,size(values,2))],[],'all');
            speedViolation=max([abs(values(7:12,:))-r.physical.actuator.ldotMax;zeros(1,size(values,2))],[],'all');
            singularViolation=max([r.strict.sigmaSafe^2-values(19,:),0]);
            forceViolation=max([reshape(r.strict.forceMin-U,[],1);reshape(U-r.strict.forceMax,[],1);0]);
            rateViolation=max([reshape(abs(diff([c(31:36),U],1,2))/r.c.Ts-r.strict.forceRateLimit,[],1);0]);
            report.rows{end+1}=struct('model',labels{model},'selection_index',index,'candidate',d.candidate(i),'mode',char(mode), ...
                'maximum_acceleration_mps2',peak,'acceleration_violation_mps2',max([reshape(acceleration-ch5PredictionAccelerationLimit(r),[],1);0]), ...
                'length_violation_m',lengthViolation,'speed_violation_mps',speedViolation,'singularity_violation',singularViolation, ...
                'command_force_violation_N',forceViolation,'command_rate_violation_Nps',rateViolation,'peak_leg',leg,'peak_trace_point',point);
        end
    end
end
report.completed=true;save(fullfile(directory,'raw_audit_v65.mat'),'report','-v7');
fid=fopen(fullfile(directory,'raw_audit_v65.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
