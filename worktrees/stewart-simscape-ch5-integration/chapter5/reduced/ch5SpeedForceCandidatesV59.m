function report=ch5SpeedForceCandidatesV59()
% Same 24 previously fixed validation windows, one new pilot seed, four candidates.
[~,root]=ch5Init();inputDirectory=fullfile(root,'chapter5','artifacts','control_force_v57','pilot');
directory=fullfile(inputDirectory,'candidate_v59');if ~isfolder(directory),mkdir(directory);end
assert(~isfile(fullfile(directory,'candidate_report.mat')),'ch5:ExistingAttempt','Preserve candidate attempts.');
s=load(fullfile(inputDirectory,'selection.mat'));data=load(fullfile(inputDirectory,'candidates_5101.mat'));w=load(fullfile(inputDirectory,'seed_5101','weights.mat'));
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'completed',false,'formal_ready',false,'rows',{{}},'parity',[],'error','');
try
    golden=ch5Denoise(double(w.golden_z),double(w.golden_context),double(w.golden_t),w);
    aa=double(w.alpha_bar(double(w.golden_t)+1))';clean=sqrt(aa).*double(w.golden_z)-sqrt(1-aa).*golden;
    goldContext=double(w.golden_context).*double(w.context_std(:))+double(w.context_mean(:));
    [~,sampled]=ch5ForceSampleV57(goldContext,zeros(6,1),w,double(w.golden_z));
    report.parity=struct('v_max',max(abs(golden-double(w.golden_v)),[],'all'),'clean_max',max(abs(clean-double(w.golden_clean)),[],'all'), ...
        'sample_max',max(abs(sampled-double(w.golden_sample)),[],'all'));
    assert(max([report.parity.v_max,report.parity.clean_max,report.parity.sample_max])<1e-4,'ch5:ForceParity','MATLAB/Python inference mismatch.');
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepareV48(4,mode);r.reduced=struct('velocity_weight',1,'factor_derivatives',false,'linearization_collision_reserve',.0002, ...
            'reuse_sensitivities',false,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);k=ch5ReducedKernelsV4(r);
        for index=unique(data.selection_index)
            c=double(s.context(:,index));if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
            x=c(1:r.plant.n);previous=c(31:36);goal=c(37:42);o=reshape(c(43:218),22,8);
            ids=find(data.selection_index==index);priors=reshape(double(data.residual(:,ids)),6,60,4)+previous;
            rawTeacher=previous+cumsum(reshape(double(s.delta(:,index)),6,60),2);
            brake=ch5ReducedBrakeSequence(r);inc=full(brake(x,previous));[incCheck,incTraj]=ch5ForceAuditV58(x,previous,inc,goal,o,r,k,[]);
            if ~incCheck.accepted,[inc,~]=ch5ReducedTerminalV23(x,inc,r,k,incTraj.X);[incCheck,incTraj]=ch5ForceAuditV58(x,previous,inc,goal,o,r,k,[]);end
            timer=tic;[proposals,poses,solves]=ch5ForceStepV59(x,previous,inc,goal,o,priors,r,k,0,incTraj.artificial_pose);projectSeconds=toc(timer);
            for b=1:4
                [rawCheck,rawTraj,rawCost]=ch5ForceAuditV58(x,previous,priors(:,:,b),goal,o,r,k,[]); %#ok<ASGLU>
                projectedCheck=struct('accepted',false);projectedTraj=[];projectedCost=inf;lineUsed=-1;
                if solves{b}.finite
                    for line=0:3
                        a=2^(-line);trial=inc+a*(proposals(:,:,b)-inc);qs=incTraj.artificial_pose+a*(poses(:,b)-incTraj.artificial_pose);
                        [projectedCheck,projectedTraj,projectedCost]=ch5ForceAuditV58(x,previous,trial,goal,o,r,k,qs);
                        lineUsed=line;if projectedCheck.accepted,break;end
                    end
                end
                report.rows{end+1}=struct('selection_index',index,'window_index',s.window_index(index),'mode',char(mode), ...
                    'seed',5101,'candidate',b,'raw_check',rawCheck,'projected_check',projectedCheck,'raw_cost',rawCost,'projected_cost',projectedCost, ...
                    'incumbent_check',incCheck,'solve',solves{b},'line',lineUsed,'batch_projection_seconds',projectSeconds, ...
                    'force_rms_error_N',sqrt(mean((priors(:,:,b)-rawTeacher).^2,'all')), ...
                    'increment_rms_error_N',sqrt(mean(diff([zeros(6,1),priors(:,:,b)-rawTeacher],1,2).^2,'all')));
            end
            write();fprintf('V57 candidates checked window%d %s\n',s.window_index(index),mode);
        end
    end
    report.completed=true;write();
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write();rethrow(ex);end
    function write()
        save(fullfile(directory,'candidate_report.mat'),'report','-v7');fid=fopen(fullfile(directory,'candidate_report.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    end
end
