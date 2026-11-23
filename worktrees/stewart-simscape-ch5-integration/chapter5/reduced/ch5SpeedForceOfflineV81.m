function report=ch5SpeedForceOfflineV81()
% Audit every saved generated prior after execution, including rejected draws.
% This restores raw denominators without adding diagnostic cost to control.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','online_v80');
if ~isfolder(directory),mkdir(directory);end
assert(~isfile(fullfile(directory,'offline_raw_v81.json')),'ch5:ExistingAttempt','Preserve offline audit attempts.');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for j=1:numel(names),exists(j)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{j}));if exists(j),saved{j}=evalin('base',names{j});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('completed',false,'pid',feature('getpid'),'formal',false,'rows',{{}},'episodes',{{}},'error','','telemetry_correction','Empty generated arrays in V77-V80 have zero candidates; raw arrays define the denominator.');
try
for version=[77,78,79,80,81]
    files=dir(fullfile(fileparts(directory),sprintf('full_v%d',version),'*.mat'));
    for fi=1:numel(files)
        file=fullfile(files(fi).folder,files(fi).name);source=load(file,'predictions');
        mode="ideal-force";if contains(files(fi).name,'nonideal'),mode="nonideal-force";end
        r=ch5ReducedPrepareV48(4,mode);k=ch5ForceKernelsV64(r);begin=numel(report.rows);timer=tic;
        for pi=1:numel(source.predictions)
            p=source.predictions{pi}.prediction;if ~isfield(p,'raw_generated_count')||~isfield(p,'raw_generated')||isempty(p.raw_generated),continue;end
            ch5ForceTerminalV62();
            for b=1:size(p.raw_generated,3)
                if version>=80,check=ch5ForceAuditV80(p.raw_state,p.raw_previous,p.raw_generated(:,:,b),p.goal,p.observed,r,k,[]);
                else,check=ch5ForceAuditV77(p.raw_state,p.raw_previous,p.raw_generated(:,:,b),p.goal,p.observed,r,k,[]);end
                row=struct('version',version,'episode',files(fi).name,'time',source.predictions{pi}.time,'candidate',b, ...
                    'accepted',check.accepted,'internal_physics_passed',check.finite&&check.internal_check.accepted&&check.command_rate_violation<=1e-6, ...
                    'terminal_region_ratio',check.terminal_region_ratio,'inequality_violation',check.inequality_violation, ...
                    'selected_source',p.selection_source,'raw_was_selected',strcmp(p.selection_source,'raw_prior')&&p.chosen_candidate==b);
                report.rows{end+1}=row;
            end
        end
        added=report.rows(begin+1:end);episode=struct('version',version,'episode',files(fi).name,'generated',numel(added), ...
            'accepted',sum(cellfun(@(z)z.accepted,added)),'seconds',toc(timer));report.episodes{end+1}=episode;
        writeJSON(fullfile(directory,'offline_raw_v81.json'),report);
        fprintf('V%d offline raw %s %d/%d accepted elapsed%.1fs\n',version,files(fi).name,episode.accepted,episode.generated,episode.seconds);
    end
end
report.completed=true;writeJSON(fullfile(directory,'offline_raw_v81.json'),report);
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');writeJSON(fullfile(directory,'offline_raw_v81.json'),report);rethrow(ex);end
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
