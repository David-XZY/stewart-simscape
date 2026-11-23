function report=ch5SpeedForceLogicV77()
% Verify raw diagnostic toggling cannot change a decision; preserve fault tests.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','online_v77');
assert(~isfile(fullfile(directory,'logic.json')),'ch5:ExistingAttempt','Preserve previous verification.');
gate=jsondecode(fileread(fullfile(directory,'parity.json')));assert(gate.accepted,'ch5:Parity','Require numerical checks first.');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for j=1:numel(names),exists(j)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{j}));if exists(j),saved{j}=evalin('base',names{j});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('completed',false,'pid',feature('getpid'),'formal',false,'rows',{{}},'fault',[],'error','');
try
for mode=["nonideal-force","ideal-force"]
    for index=[1,4]
        r=ch5ReducedPrepareV48(index,mode);r=configure(r,root);r.reduced.reuse_sensitivities=false;
        x=[r.task.q0;r.task.qd0;repmat(r.initialForce,(r.plant.n-12)/6,1)];[goal,o]=ch5Observe(r.task,0);
        outputs=cell(1,2);decisions=cell(1,2);times=zeros(1,2);
        for flag=0:1
            if isappdata(0,'ch5ReducedExecution'),rmappdata(0,'ch5ReducedExecution');end
            if isappdata(0,'ch5ReducedSensitivity'),rmappdata(0,'ch5ReducedSensitivity');end
            ch5ForceTerminalV62();setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',12345));r.reduced.audit_raw_candidates=logical(flag);
            timer=tic;[outputs{flag+1},~,decisions{flag+1}]=ch5ForceControlV77(x,r.initialForce,goal,o,r,0);times(flag+1)=toc(timer);
        end
        a=decisions{1};b=decisions{2};row=struct('mode',char(mode),'task',index,'force_difference_N',max(abs(outputs{1}-outputs{2})), ...
            'source_matches',strcmp(a.selection_source,b.selection_source),'candidate_matches',a.chosen_candidate==b.chosen_candidate, ...
            'generated',a.raw_generated_count,'solves',numel(a.denoise_attempts),'without_diagnostics_seconds',times(1), ...
            'with_diagnostics_seconds',times(2),'raw_audited_without',a.raw_audited,'raw_audited_with',b.raw_audited,'source',a.selection_source);
        report.rows{end+1}=row;writeJSON(fullfile(directory,'logic.json'),report);
    end
end
r.reduced.audit_raw_candidates=false;r.reduced.force_rejection_window=[0,.2];
if isappdata(0,'ch5ReducedExecution'),rmappdata(0,'ch5ReducedExecution');end
[u,d,prediction]=ch5ForceControlV77(x,r.initialForce,goal,o,r,0);
held=getappdata(0,'ch5ReducedExecution');
report.fault=struct('used_fallback',d.usedFallback,'accepted_trajectory_cleared',isempty(held.trajectory), ...
    'full_horizon_failure_recorded',prediction.full_horizon_failure_active,'force_finite',all(isfinite(u)));
report.accepted=all(cellfun(@(z)z.force_difference_N<1e-10&&z.source_matches&&z.candidate_matches,report.rows))&& ...
    report.fault.used_fallback&&report.fault.accepted_trajectory_cleared&&report.fault.full_horizon_failure_recorded;
report.completed=true;writeJSON(fullfile(directory,'logic.json'),report);assert(report.accepted,'ch5:SelectionLogic','Diagnostic or rejection handling changed decision semantics.');
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');writeJSON(fullfile(directory,'logic.json'),report);rethrow(ex);end
end
function r=configure(r,root)
r.reduced=struct('iterations',1,'backtracking',3,'linearization_collision_reserve',.0002,'velocity_weight',1, ...
    'factor_derivatives',true,'reuse_sensitivities',true,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1, ...
    'audit_raw_candidates',false,'sensitivity_maximum_age',.3);
r.force_weights=load(fullfile(root,'chapter5','artifacts','control_force_v62','raw_v76','seed_5101','weights.mat'));
r.algorithm_revision='force-v77-lazy-candidate-computation-development';
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
