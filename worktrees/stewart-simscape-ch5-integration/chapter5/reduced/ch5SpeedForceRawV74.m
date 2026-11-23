function report=ch5SpeedForceRawV74()
% No QP, clipping, repair or candidate selection before the raw gate.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','raw_v74');
assert(~isfile(fullfile(directory,'raw_report.json')),'ch5:ExistingAttempt','Preserve attempts.');
s=load(fullfile(root,'chapter5','artifacts','control_force_v57','pilot','selection.mat'));labels=load(fullfile(root,'chapter5','artifacts','control_force_v57','pilot','repaired.mat'),'accepted','residual');
data=load(fullfile(directory,'candidates_5101.mat'));w=load(fullfile(directory,'seed_5101','weights.mat'));
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for j=1:numel(names),exists(j)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{j}));if exists(j),saved{j}=evalin('base',names{j});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('completed',false,'pid',feature('getpid'),'formal',false,'rows',{{}},'parity',[],'same_noise',false,'raw_no_projection',true);
old=load(fullfile(root,'chapter5','artifacts','control_force_v62','temporal_v64','candidates_5101.mat'),'noise','selection_index');
report.same_noise=isequal(data.noise,old.noise)&&isequal(data.selection_index,old.selection_index);assert(report.same_noise,'ch5:MatchedNoise','Require identical old draws.');
v=ch5ForceDenoiseV65(w.golden_z,w.golden_context,double(w.golden_t),w);report.parity.v_max=max(abs(double(v)-double(w.golden_v)),[],'all');
timer=tic;
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);k=ch5ForceKernelsV64(r);decoder=ch5ForceInnovationV73(r);
    for index=unique(data.selection_index)
        c=double(s.context(:,index));if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        ch5ForceTerminalV62();x=c(1:r.plant.n);previous=c(31:36);goal=c(37:42);o=reshape(c(43:218),22,8);ids=find(data.selection_index==index);
        anchor=ch5ForceTerminalV62(x(1:6),r,k);L=anchor.Du*anchor.K./anchor.scale';V=reshape(double(data.innovation(:,ids)),6,60,4);map=decoder.map(4);
        U=reshape(full(map(repmat(x,1,4),repmat(previous,1,4),reshape(V,6,240),repmat(L,1,4),repmat(anchor.xi,1,4))),6,60,4);
        [matlabU,matlabZ,info]=ch5ForceSampleV74(c,previous,w,data.noise(:,ids),r,k);
        for b=1:4
            [check,traj,cost]=ch5ForceAuditV62(x,previous,U(:,:,b),goal,o,r,k,[]);
            [~,values]=k.preview(x,U(:,:,b));values=full(values);
            physicalPassed=check.finite&&check.internal_check.accepted&&check.command_rate_violation<=1e-6;
            labelU=reshape(double(labels.residual(:,index)),6,60)+previous;
            report.rows{end+1}=struct('index',index,'mode',char(mode),'candidate',b,'check',check,'physical_passed',physicalPassed, ...
                'cost',cost,'label_accepted',logical(labels.accepted(index)),'force_error_rms_N',sqrt(mean((U(:,:,b)-labelU).^2,'all')), ...
                'maximum_acceleration_mps2',max(abs(values(13:18,:)),[],'all'),'endpoint_velocity_mps',norm(traj.X(7:9,end)), ...
                'inference_seconds',info.seconds,'network_seconds',info.network_seconds,'sample_force_parity_N',max(abs(matlabU(:,:,b)-U(:,:,b)),[],'all'), ...
                'innovation_parity_N',max(abs(double(w.force_mean(:))+double(w.force_std(:)).*matlabZ(:,b)-data.innovation(:,ids(b)))));
        end
        fprintf('V74 raw candidates index %d elapsed %.1fs\n',index,toc(timer));
    end
end
report.parity.force_max_N=max(cellfun(@(z)z.sample_force_parity_N,report.rows));report.parity.accepted=report.parity.v_max<1e-4&&report.parity.force_max_N<1e-3;
report.completed=true;report.seconds=toc(timer);save(fullfile(directory,'raw_report.mat'),'report','-v7');
fid=fopen(fullfile(directory,'raw_report.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(report.parity.accepted,'ch5:InferenceParity','Python/MATLAB full inference parity failed.');
end
