function report=ch5SpeedForceLabelsV57()
% Fixed pilot selection, unchanged raw teacher, accepted repairs, complete rejection ledger.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v57','pilot');
assert(~isfile(fullfile(directory,'repaired.mat')),'ch5:ExistingAttempt','Preserve previous label attempts.');
if isfile(fullfile(directory,'label_report.json'))
    archive=fullfile(directory,['label_failed_',char(datetime('now','Format','yyyyMMdd_HHmmss')),'.json']);
    copyfile(fullfile(directory,'label_report.json'),archive);
end
clear ch5ForceTerminalV57 ch5ForceAuditV57 ch5ForcePhysicsRowsV57 ch5ForceStepV57
s=load(fullfile(directory,'selection.mat'));context=double(s.context);split=s.split;delta=double(s.delta);count=size(context,2);
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@restore); %#ok<NASGU>
residual=nan(360,count);accepted=false(1,count);physical_J=zeros(count,12,360,'single');physical_g=zeros(12,count,'single');
report=struct('pid',feature('getpid'),'completed',false,'formal_ready',false,'rows',{{}},'regions',{{}},'error','');timer=tic;
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepareV48(4,mode);r.reduced=struct('velocity_weight',1,'factor_derivatives',false,'linearization_collision_reserve',.0002, ...
            'reuse_sensitivities',false,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);
        k=ch5ReducedKernelsV4(r);t=ch5ForceTerminalV57(r.task.q0,r,k,true);report.regions{end+1}=struct('mode',char(mode),'terminal',t);
        assert(t.numerically_valid,'ch5:TerminalRegion','Numerical terminal region development gate failed.');
        for index=1:count
            c=context(:,index);isNonideal=c(219)>.5;if isNonideal~=strcmp(mode,'nonideal-force'),continue;end
            x=c(1:r.plant.n);previous=c(31:36);goal=c(37:42);o=reshape(c(43:218),22,8);raw=previous+cumsum(reshape(delta(:,index),6,60),2);
            [rawCheck,traj,cost]=ch5ForceAuditV57(x,previous,raw,goal,o,r,k,[],raw);U=raw;check=rawCheck;repair=struct('type','none','attempts',{{}});
            if ~check.accepted
                [candidate,terminalRepair]=ch5ReducedTerminalV23(x,U,r,k,traj.X);repair.type='terminal_initializer';repair.initializer=terminalRepair;
                [candidateCheck,candidateTraj,candidateCost]=ch5ForceAuditV57(x,previous,candidate,goal,o,r,k,[],raw);
                repair.attempts{end+1}=candidateCheck;
                if candidateCheck.accepted,U=candidate;check=candidateCheck;traj=candidateTraj;cost=candidateCost;
                else
                    % A bounded constrained repair; no rejected record is discarded.
                    U=candidate;traj=candidateTraj;check=candidateCheck;
                    for iteration=1:3
                        [candidate,qs,solve]=ch5ForceStepV57(x,previous,U,goal,o,raw,r,k,0,traj.artificial_pose);
                        [candidateCheck,candidateTraj,candidateCost]=ch5ForceAuditV57(x,previous,candidate,goal,o,r,k,qs,raw);
                        repair.attempts{end+1}=struct('solve',solve,'check',candidateCheck);
                        if candidateCheck.accepted,U=candidate;check=candidateCheck;traj=candidateTraj;cost=candidateCost;repair.type='constrained_projection';break;end
                        if ~solve.finite,break;end
                        U=candidate;traj=candidateTraj;check=candidateCheck;
                    end
                end
            end
            accepted(index)=check.accepted;
            if check.accepted
                residual(:,index)=reshape(U-previous,360,1);[J,g]=ch5ForcePhysicsRowsV57(x,U,traj,r,k);
                physical_J(index,:,:)=single(reshape(J,1,12,360));physical_g(:,index)=single(g);
            end
            report.rows{end+1}=struct('selection_index',index,'window_index',s.window_index(index),'split',s.split{index}, ...
                'source',s.source{index},'task',s.task{index},'mode',char(mode),'raw_check',rawCheck,'repaired_check',check, ...
                'repair',repair,'maximum_force_change_N',max(abs(U-raw),[],'all'),'force_rms_change_N',sqrt(mean((U-raw).^2,'all')), ...
                'cost',cost,'terminal_pose',traj.artificial_pose);
            if mod(numel(report.rows),32)==0,write();fprintf('V57 labels %d/%d accepted %d elapsed %.1fs\n',numel(report.rows),count,nnz(accepted),toc(timer));end
        end
    end
    report.completed=true;report.seconds=toc(timer);report.accepted=nnz(accepted);report.rejected=count-nnz(accepted);
    save(fullfile(directory,'repaired.mat'),'context','delta','split','residual','accepted','physical_J','physical_g','s','report','-v7');write();
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write();rethrow(ex);end
    function write()
        fid=fopen(fullfile(directory,'label_report.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    end
    function restore()
        for ii=1:numel(names),if exists(ii),assignin('base',names{ii},saved{ii});else,evalin('base',['clear ',names{ii}]);end;end
    end
end
