function report=ch5SpeedCandidateFailureDetailsV55()
% Diagnose preserved V39 candidates without resampling or changing controllers.
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');
directory=fullfile(base,'candidate_failure_details_v55');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
sourceFile=fullfile(base,'teacher_candidate_audit_v39','audit.mat');source=load(sourceFile,'report');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names)
    exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));
    if exists(i),saved{i}=evalin('base',names{i});end
end
cleanup=onCleanup(@restore); %#ok<NASGU>
report=struct('pid',feature('getpid'),'source',sourceFile,'source_sha256',digest(sourceFile), ...
    'completed',false,'error','','rows',{{}},'limits',{{}});
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepare(4,mode);r.reduced=struct('velocity_weight',1,'factor_derivatives',false);k=ch5ReducedKernelsV4(r);
        report.limits{end+1}=struct('mode',char(mode),'length_min_m',r.physical.lmin,'length_max_m',r.physical.lmax, ...
            'leg_speed_limit_m_s',r.physical.actuator.ldotMax,'leg_acceleration_prediction_limit_m_s2',ch5PredictionAccelerationLimit(r), ...
            'leg_acceleration_physical_limit_m_s2',r.physical.actuator.lddotMax,'sigma_lower_squared',r.strict.sigmaSafe^2, ...
            'force_min_N',r.strict.forceMin,'force_max_N',r.strict.forceMax,'command_rate_limit_N_s',r.strict.forceRateLimit);
        for j=1:numel(source.report.rows)
            row=source.report.rows{j};if ~strcmp(row.mode,mode),continue;end
            c=row.context;x=c(1:r.plant.n);previous=c(31:36);
            [~,values]=k.preview(x,row.teacher_U);teacher=details(full(values),r);
            result=struct('window_index',row.window_index,'mode',row.mode,'teacher',teacher,'generated',{{}});
            for ci=1:numel(row.generated)
                candidate=row.generated{ci};[nodes,values]=k.preview(x,candidate.U);nodes=full(nodes);
                d=details(full(values),r);D=diff([previous,candidate.U],1,2);
                eu=candidate.U-row.teacher_U;ed=diff([zeros(6,1),eu],1,2);
                entry=struct('seed',candidate.seed,'candidate',candidate.candidate,'mechanics',d, ...
                    'old_check',candidate.check,'command_peak_N',max(abs(candidate.U),[],'all'), ...
                    'command_rate_peak_N_s',max(abs(D)/r.c.Ts,[],'all'),'terminal_velocity',nodes(7:12,end), ...
                    'increment_rms_error_N',sqrt(mean(ed.^2,'all')),'force_rms_error_N',sqrt(mean(eu.^2,'all')), ...
                    'last_force_rms_error_N',sqrt(mean(eu(:,end).^2)), ...
                    'mean_increment_error_rms_N',sqrt(mean(mean(ed,2).^2)));
                result.generated{end+1}=entry;
            end
            report.rows{end+1}=result;write();
            fprintf('Candidate failure details: %s window %d, %d stored samples\n',mode,row.window_index,numel(result.generated));
        end
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
        save(fullfile(directory,'audit.mat'),'report','-v7');
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    end
end

function d=details(values,r)
lower=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2;r.strict.forceMin];
upper=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf;r.strict.forceMax];
errors=max(max(lower-values,values-upper),0);
groups={1:6,7:12,13:18,19,20:25};names={'length','speed','acceleration','singularity_squared','actual_force'};
times=reshape(r.plant.traceTime(:)+(0:r.c.H-1)*r.c.Ts,1,[]);
assert(size(values,1)==25&&size(values,2)==numel(times),'ch5:DiagnosticShape','Unexpected preview layout.');
finite=all(isfinite(values),1);d=struct('finite',all(finite),'samples',size(values,2),'finite_samples',sum(finite));
for i=1:numel(groups)
    e=errors(groups{i},:);mask=any(e>1e-6,1)&all(isfinite(values(groups{i},:)),1);first=find(mask,1);
    if isempty(first),time=inf;else,time=times(first);end
    d.(names{i})=struct('violated',any(mask),'max_violation',max(e(:,finite),[],'all'),'first_violation_seconds',time);
end
first=find(any(errors>1e-6,1)&finite,1);
if isempty(first),d.first_mechanical_violation_seconds=inf;d.first_mechanical_types={};
else
    d.first_mechanical_violation_seconds=times(first);
    d.first_mechanical_types=names(cellfun(@(ids)any(errors(ids,first)>1e-6),groups));
end
d.leg_speed_peak_m_s=max(abs(values(7:12,finite)),[],'all');
d.leg_acceleration_peak_m_s2=max(abs(values(13:18,finite)),[],'all');
d.sigma_lower_squared_min=min(values(19,finite));
end

function out=digest(file)
fid=fopen(file,'rb');h=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),h.update(fread(fid,1048576,'*uint8'));end
fclose(fid);out=lower(reshape(dec2hex(typecast(h.digest(),'uint8'),2).',1,[]));
end
