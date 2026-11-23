function report=ch5SpeedForceParityV81()
% Fixed draws: numerical refactors must preserve admission and objective.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','online_v81');if ~isfolder(directory),mkdir(directory);end
assert(~isfile(fullfile(directory,'parity.json')),'ch5:ExistingAttempt','Preserve previous verification.');
s=load(fullfile(root,'chapter5','artifacts','control_force_v57','pilot','selection.mat'));
data=load(fullfile(root,'chapter5','artifacts','control_force_v62','raw_v80','candidates_5101.mat'));
w=load(fullfile(root,'chapter5','artifacts','control_force_v62','raw_v80','seed_5101','weights.mat'));
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for j=1:numel(names),exists(j)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{j}));if exists(j),saved{j}=evalin('base',names{j});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('completed',false,'pid',feature('getpid'),'formal',false,'rows',{{}},'qp',{{}},'old_solve_seconds',[],'new_solve_seconds',[],'error','');timer=tic;
try
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);k=ch5ForceKernelsV64(r);
    r.reduced=struct('linearization_collision_reserve',.0002,'factor_derivatives',true, ...
        'reuse_sensitivities',false,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);
    first=true;
    for index=unique(data.selection_index)
        c=double(s.context(:,index));if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        ch5ForceTerminalV62();x=c(1:r.plant.n);previous=c(31:36);goal=c(37:42);o=reshape(c(43:218),22,8);ids=find(data.selection_index==index);
        [U,~,sampleInfo]=ch5ForceSampleV78(c,previous,w,data.noise(:,ids),r,k);
        [pair,~,pairInfo]=ch5ForceSampleV81(c,previous,w,data.noise(:,ids),r,k);
        pairError=max(abs(U-pair),[],'all');
        for b=1:4
            oldTimer=tic;[oldCheck,~,oldCost]=ch5ForceAuditV80(x,previous,U(:,:,b),goal,o,r,k,[]);oldSeconds=toc(oldTimer);
            newTimer=tic;[newCheck,~,newCost]=ch5ForceAuditV80(x,previous,pair(:,:,b),goal,o,r,k,[]);newSeconds=toc(newTimer);
            costError=0;if isfinite(oldCost)||isfinite(newCost),costError=abs(newCost-oldCost);end
            violationError=0;if isfinite(oldCheck.inequality_violation)||isfinite(newCheck.inequality_violation),violationError=abs(oldCheck.inequality_violation-newCheck.inequality_violation);end
            report.rows{end+1}=struct('index',index,'mode',char(mode),'candidate',b,'old_accepted',oldCheck.accepted, ...
                'new_accepted',newCheck.accepted,'cost_error',costError,'violation_error',violationError, ...
                'pair_force_error_N',pairError,'old_audit_seconds',oldSeconds,'new_audit_seconds',newSeconds, ...
                'four_sampling_seconds',sampleInfo.seconds,'two_sampling_seconds',pairInfo.seconds);
        end
        if first
            brake=ch5ReducedBrakeSequence(r);incumbent=full(brake(x,previous));nodes=full(k.nodes(x,incumbent));qs=nodes(1:6,end);
            oldPrepared=ch5ForcePrepareStepV80(x,previous,incumbent,goal,o,U(:,:,1),r,k,0,qs);
            newPrepared=ch5ForcePrepareStepV81(x,previous,incumbent,goal,o,U(:,:,1),r,k,0,qs);oldWarm=[];newWarm=[];
            for b=1:4
                oldTimer=tic;[oldU,oldQs,oldInfo,oldWarm]=ch5ForceSolveStepV77(oldPrepared,U(:,:,b),oldWarm);oldSeconds=toc(oldTimer);
                newTimer=tic;[newU,newQs,newInfo,newWarm]=ch5ForceSolveStepV77(newPrepared,U(:,:,b),newWarm);newSeconds=toc(newTimer);
                report.old_solve_seconds(end+1)=oldSeconds;report.new_solve_seconds(end+1)=newSeconds;
                report.qp{end+1}=struct('mode',char(mode),'candidate',b,'old_finite',oldInfo.finite,'new_finite',newInfo.finite, ...
                    'force_error_N',max(abs(oldU-newU),[],'all'),'steady_pose_error',max(abs(oldQs-newQs)));
            end
            first=false;
        end
        fprintf('V81 native scan parity %s index%d rows%d elapsed%.1fs\n',mode,index,numel(report.rows),toc(timer));
        writeJSON(fullfile(directory,'parity.json'),report);
    end
end
report.admission_matches=all(cellfun(@(z)z.old_accepted==z.new_accepted,report.rows));
report.maximum_cost_error=max(cellfun(@(z)z.cost_error,report.rows));
report.maximum_violation_error=max(cellfun(@(z)z.violation_error,report.rows));
report.maximum_pair_force_error_N=max(cellfun(@(z)z.pair_force_error_N,report.rows));
report.qp_matches=all(cellfun(@(z)z.old_finite==z.new_finite&&z.force_error_N<1e-6&&z.steady_pose_error<1e-9,report.qp));
report.accepted=report.admission_matches&&report.maximum_cost_error<1e-8&&report.maximum_violation_error<1e-9&&report.maximum_pair_force_error_N<1e-3&&report.qp_matches;
report.completed=true;report.seconds=toc(timer);writeJSON(fullfile(directory,'parity.json'),report);
assert(report.accepted,'ch5:Parity','Numerical refactor changed admission, cost or shared QP.');
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');writeJSON(fullfile(directory,'parity.json'),report);rethrow(ex);end
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
