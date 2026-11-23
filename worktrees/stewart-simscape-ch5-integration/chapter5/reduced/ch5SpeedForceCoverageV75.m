function report=ch5SpeedForceCoverageV75()
% Verify which constraint classes the old top-11 loss actually saw.
[~,root]=ch5Init();out=fullfile(root,'chapter5','artifacts','control_force_v62','raw_v75');assert(~isfile(fullfile(out,'coverage_audit.json')),'ch5:ExistingAttempt','Preserve audits.');
d=load(fullfile(root,'chapter5','artifacts','control_force_v57','pilot','repaired.mat'),'context','residual','accepted','split');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for j=1:numel(names),exists(j)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{j}));if exists(j),saved{j}=evalin('base',names{j});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('completed',false,'pid',feature('getpid'),'formal',false,'rows',{{}});
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);k=ch5ForceKernelsV64(r);n=r.plant.n;H=60;
    for index=find(d.accepted & strcmp(d.split,'validation'))
        c=d.context(:,index);if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        U=reshape(double(d.residual(:,index)),6,H)+c(31:36);X=full(k.nodes(c(1:n),U));states=reshape([X(:,1:H);X(:,2:end)],n,2*H);values=full(k.mechanics(states,repelem(U,1,2)));
        lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2];hi=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf];
        sc=[ones(6,1);r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;r.strict.sigmaSafe^2];hiIndex=find(isfinite(hi));g=[(values(hiIndex,:)-hi(hiIndex))./sc(hiIndex);(lo-values)./sc];
        [~,order]=sort(g(:),'descend');[row,~]=ind2sub(size(g),order(1:11));mechanical=row;lower=row>numel(hiIndex);mechanical(lower)=row(lower)-numel(hiIndex);mechanical(~lower)=hiIndex(row(~lower));
        report.rows{end+1}=struct('index',index,'mode',char(mode),'legacy_length_rows',nnz(mechanical<=6), ...
            'legacy_speed_rows',nnz(mechanical>6&mechanical<=12),'legacy_acceleration_rows',nnz(mechanical>12&mechanical<=18), ...
            'legacy_singularity_rows',nnz(mechanical==19),'new_length_rows',12,'new_speed_rows',12,'new_acceleration_rows',12,'new_singularity_rows',1);
    end
end
report.completed=true;fid=fopen(fullfile(out,'coverage_audit.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
