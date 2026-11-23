function report=ch5SpeedForceResidualV75()
% Preserve and explain every remaining raw rejection, including bad labels.
[~,root]=ch5Init();out=fullfile(root,'chapter5','artifacts','control_force_v62','raw_v75');assert(~isfile(fullfile(out,'residual_audit.json')),'ch5:ExistingAttempt','Preserve residual audits.');
a=jsondecode(fileread(fullfile(out,'raw_report.json')));s=load(fullfile(root,'chapter5','artifacts','control_force_v57','pilot','selection.mat'));d=load(fullfile(out,'candidates_5101.mat'));
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for j=1:numel(names),exists(j)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{j}));if exists(j),saved{j}=evalin('base',names{j});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('completed',false,'pid',feature('getpid'),'formal',false,'rows',{{}});
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);k=ch5ForceKernelsV64(r);decoder=ch5ForceInnovationV73(r);
    for ai=1:numel(a.rows)
        row=a.rows(ai);if row.check.accepted||~strcmp(mode,row.mode),continue;end
        c=double(s.context(:,row.index));x=c(1:r.plant.n);previous=c(31:36);ch5ForceTerminalV62();t=ch5ForceTerminalV62(x(1:6),r,k);L=t.Du*t.K./t.scale';
        id=find(d.selection_index==row.index&d.candidate==row.candidate);U=full(decoder(x,previous,reshape(double(d.innovation(:,id)),6,60),L,t.xi));
        initial=full(r.plant.mechanicalValues(x,previous));[~,v]=k.preview(x,U);v=full(v);acc=abs(v(13:18,:));[peak,point]=max(acc,[],'all','linear');[leg,point]=ind2sub(size(acc),point);
        brake=ch5ReducedBrakeSequence(r);brakeU=full(brake(x,previous));[brakeCheck,~]=ch5ForceAuditV62(x,previous,brakeU,c(37:42),reshape(c(43:218),22,8),r,k,[]);
        report.rows{end+1}=struct('index',row.index,'candidate',row.candidate,'mode',char(mode),'label_accepted',row.label_accepted, ...
            'initial_maximum_acceleration_mps2',max(abs(initial(13:18))),'initial_prediction_acceleration_violation',max(0,max(abs(initial(13:18)))-ch5PredictionAccelerationLimit(r)), ...
            'initial_hard_acceleration_violation',max(0,max(abs(initial(13:18)))-r.physical.actuator.lddotMax), ...
            'raw_maximum_acceleration_mps2',peak,'peak_leg',leg,'peak_internal_sample',point,'physical_passed',row.physical_passed,'raw_check',row.check,'brake_check',brakeCheck);
    end
end
report.completed=true;fid=fopen(fullfile(out,'residual_audit.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
