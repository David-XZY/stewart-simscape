function report=ch5SpeedForceConditionV76()
% Identical observed moving boxes/noise, original vs neutralized random columns.
[~,root]=ch5Init();out=fullfile(root,'chapter5','artifacts','control_force_v62','raw_v76');assert(~isfile(fullfile(out,'conditioning_audit.json')),'ch5:ExistingAttempt','Preserve attempts.');
d=load(fullfile(out,'conditioning_probe.mat'));w=load(fullfile(out,'seed_5101','weights.mat'));names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for j=1:numel(names),exists(j)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{j}));if exists(j),saved{j}=evalin('base',names{j});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('completed',false,'pid',feature('getpid'),'formal',false,'rows',{{}},'maximum_force_parity_N',0,'full_observation_constraints_retained',true);
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);k=ch5ForceKernelsV64(r);decoder=ch5ForceInnovationV73(r);
    for i=1:size(d.context,2)
        c=double(d.context(:,i));if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        x=c(1:r.plant.n);previous=c(31:36);ch5ForceTerminalV62();t=ch5ForceTerminalV62(x(1:6),r,k);L=t.Du*t.K./t.scale';ids=4*(i-1)+(1:4);
        [matlabU,~,info]=ch5ForceSampleV76(c,previous,w,d.noise(:,ids),r,k);values={d.original_innovation,d.neutralized_innovation};
        for variant=1:2
            V=reshape(double(values{variant}(:,ids)),6,60,4);map=decoder.map(4);U=reshape(full(map(repmat(x,1,4),repmat(previous,1,4),reshape(V,6,240),repmat(L,1,4),repmat(t.xi,1,4))),6,60,4);
            if variant==2,report.maximum_force_parity_N=max(report.maximum_force_parity_N,max(abs(U-matlabU),[],'all'));end
            for b=1:4
                [check,~]=ch5ForceAuditV62(x,previous,U(:,:,b),c(37:42),reshape(c(43:218),22,8),r,k,[]);
                report.rows{end+1}=struct('index',d.selection_index(i),'candidate',b,'mode',char(mode),'neutralized',variant==2,'check',check, ...
                    'unsupported_condition_indices',info.unsupported_condition_indices,'unobserved_conditions_learned',false);
            end
        end
    end
end
report.completed=true;report.parity_accepted=report.maximum_force_parity_N<1e-3;
fid=fopen(fullfile(out,'conditioning_audit.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(report.parity_accepted,'ch5:InferenceParity','Require MATLAB/Python full inference parity.');
end
