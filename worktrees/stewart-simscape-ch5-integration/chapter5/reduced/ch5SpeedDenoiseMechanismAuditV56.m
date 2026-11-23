function report=ch5SpeedDenoiseMechanismAuditV56()
% Check paired common-noise ablations using unchanged V23 physical/terminal gates.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'denoise_mechanism_v56');
file=fullfile(directory,'physical_summary.json');assert(~isfile(file),'ch5:ExistingAudit','Preserve previous attempts.');
data=load(fullfile(directory,'candidates.mat'));source=load(fullfile(base,'teacher_candidate_audit_v39','audit.mat'),'report');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names)
    exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));
    if exists(i),saved{i}=evalin('base',names{i});end
end
cleanup=onCleanup(@restore); %#ok<NASGU>
report=struct('pid',feature('getpid'),'completed',false,'error','','rows',{{}});
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepare(4,mode);r.reduced=struct('velocity_weight',1,'factor_derivatives',false);k=ch5ReducedKernelsV4(r);
        for j=1:numel(data.row_ids)
            row=source.report.rows{data.row_ids(j)};if ~strcmp(row.mode,mode),continue;end
            c=row.context;x=c(1:r.plant.n);previous=c(31:36);goal=c(37:42);o=reshape(c(43:218),22,8);
            for variant=["original20","identity99_20","dense100"]
                delta=reshape(data.(variant)(:,j),6,r.c.H);U=previous+cumsum(delta,2);last=previous;
                for h=1:r.c.H
                    U(:,h)=min(max(U(:,h),max(r.strict.forceMin,last-r.strict.forceRateLimit*r.c.Ts)), ...
                        min(r.strict.forceMax,last+r.strict.forceRateLimit*r.c.Ts));last=U(:,h);
                end
                [~,check,~,cost]=ch5ReducedAuditV23(x,previous,U,goal,o,r,k,zeros(size(U)),[],false);
                eu=U-row.teacher_U;ed=diff([zeros(6,1),eu],1,2);
                report.rows{end+1}=struct('mode',char(mode),'window_index',row.window_index, ...
                    'seed',data.seed_ids(j),'candidate',data.candidate_ids(j),'variant',char(variant), ...
                    'check',check,'cost',cost,'increment_rms_error_N',sqrt(mean(ed.^2,'all')), ...
                    'force_rms_error_N',sqrt(mean(eu.^2,'all')));
            end
        end
        write();fprintf('Paired denoising mechanics complete: %s\n',mode);
    end
    report.completed=true;
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write();rethrow(ex);end
write();
    function restore()
        for ii=1:numel(names)
            if exists(ii),assignin('base',names{ii},saved{ii});else,evalin('base',['clear ',names{ii}]);end
        end
    end
    function write()
        save(fullfile(directory,'physical_audit.mat'),'report','-v7');
        fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    end
end
