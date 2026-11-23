function report=ch5SpeedTeacherCandidateAuditV39()
% Uniform held-out windows: teacher physics, terminal mismatch and real samples.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'teacher_candidate_audit_v39');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
teacher=fullfile(root,'chapter5','artifacts','teacher');context=h5read(fullfile(teacher,'validation_windows.h5'),'/context');
labels=h5read(fullfile(teacher,'validation_windows.h5'),'/delta');source=load(fullfile(teacher,'validation_windows_provenance.mat'),'provenance');
report=struct('pid',feature('getpid'),'rows',{{}},'completed',false,'error','','selection','12 fixed-random validation windows per actuator profile; no outcome filtering');
random=RandStream('mt19937ar','Seed',6201);
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepare(4,mode);r.reduced=struct('velocity_weight',1,'factor_derivatives',false);k=ch5ReducedKernelsV4(r);
        eligible=find(context(end,:)==double(mode=="nonideal-force"));selected=eligible(randperm(random,numel(eligible),12));
        for index=selected
            c=double(context(:,index));x=c(1:r.plant.n);previous=c(31:36);goal=c(37:42);o=reshape(c(43:218),22,8);
            delta=reshape(double(labels(:,index)),6,r.c.H);U=previous+cumsum(delta,2);
            [~,rawCheck,~,rawCost,rawAudit]=ch5ReducedAuditV23(x,previous,U,goal,o,r,k,zeros(size(U)),[],false);
            [repaired,repairedCheck,~,repairedCost,repairedAudit]=ch5ReducedAuditV23(x,previous,U,goal,o,r,k,zeros(size(U)));
            provenance=source.provenance.sources{index};original=load(fullfile(teacher,provenance.run),'records');first=provenance.first_control_index;
            actual=cell2mat(cellfun(@(v)v.x,original.records(first:first+r.c.H),'UniformOutput',false));predicted=full(k.nodes(x,U));
            row=struct('window_index',index,'mode',char(mode),'source',provenance,'context',c,'teacher_U',U, ...
                'teacher_raw_check',rawCheck,'teacher_repaired_check',repairedCheck,'teacher_raw_cost',rawCost, ...
                'teacher_repaired_cost',repairedCost,'teacher_repair_max_force_change',norm(repaired-U,inf), ...
                'teacher_raw_audit',rawAudit,'teacher_repaired_audit',repairedAudit,'generated',{{}}, ...
                'teacher_maximum_position_prediction_error',max(vecnorm(predicted(1:3,:)-actual(1:3,:))), ...
                'teacher_maximum_velocity_prediction_error',max(vecnorm(predicted(7:9,:)-actual(7:9,:))));
            for seed=5101:5105
                r.seed=seed;r.weights=load(fullfile(root,'chapter5','artifacts','models',sprintf('seed_%d.mat',seed)));
                generator=RandStream('mt19937ar','Seed',seed+index+620000);setappdata(0,'ch5Random',generator);
                [prior,priorInfo,candidates]=ch5ReducedPriorCandidatesV31(x,previous,goal,o,r,k);
                for ci=1:numel(candidates)
                    proposal=candidates{ci}.U;
                    [~,check,~,cost,audit]=ch5ReducedAuditV23(x,previous,proposal,goal,o,r,k,prior,[],false);
                    row.generated{end+1}=struct('seed',seed,'candidate',ci,'U',proposal,'check',check,'cost',cost,'audit',audit, ...
                        'increment_rms_error',sqrt(mean((candidates{ci}.delta-delta).^2,'all')), ...
                        'force_rms_error',sqrt(mean((proposal-U).^2,'all')),'prior',priorInfo);
                end
            end
            report.rows{end+1}=row;
            fprintf('Validation window%d %s teacher raw%d repair%d generated%d/20 pos %.3fmm\n',index,mode, ...
                rawCheck.accepted,repairedCheck.accepted,sum(cellfun(@(v)v.check.accepted,row.generated)),1000*row.teacher_maximum_position_prediction_error);
            write();
        end
    end
    report.completed=true;
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write();rethrow(ex);end
write();
    function write()
        save(fullfile(directory,'audit.mat'),'report','-v7');summary=report;
        for j=1:numel(summary.rows)
            summary.rows{j}=rmfield(summary.rows{j},{'context','teacher_U'});
            for t=1:numel(summary.rows{j}.generated),summary.rows{j}.generated{t}=rmfield(summary.rows{j}.generated{t},'U');end
        end
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(summary));fclose(fid);
    end
end
