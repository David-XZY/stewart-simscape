function report=ch5SpeedForceLabelSelectV65()
% Multi-start minimum-distortion projection, full gate on every alternative.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','labels_v65');
if ~isfolder(directory),mkdir(directory);end
assert(~isfile(fullfile(directory,'repaired.mat')),'ch5:ExistingAttempt','Preserve label selection audit.');
old=load(fullfile(root,'chapter5','artifacts','control_force_v57','pilot','repaired.mat'),'residual','accepted');
d=load(fullfile(fileparts(directory),'labels_v64','repaired.mat'),'context','delta','residual','accepted','indices','split');
context=d.context;delta=d.delta;split=d.split;indices=d.indices;count=numel(indices);residual=nan(360,count);accepted=false(1,count);
physical_J=zeros(count,12,360,'single');physical_g=zeros(12,count,'single');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'completed',false,'formal',false,'rows',{{}},'criterion','mean(force_error/20)^2 + mean(increment_error/2)^2');
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);r.reduced=struct('factor_derivatives',true);k=ch5ForceKernelsV64(r);
    for i=1:count
        c=context(:,i);if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        ch5ForceTerminalV62();x=c(1:r.plant.n);previous=c(31:36);goal=c(37:42);o=reshape(c(43:218),22,8);
        raw=previous+cumsum(reshape(delta(:,i),6,60),2);options={reshape(old.residual(:,indices(i)),6,60)+previous,reshape(d.residual(:,i),6,60)+previous};
        checks=cell(1,2);scores=inf(1,2);trajectories=cell(1,2);
        for j=1:2
            if any(~isfinite(options{j}),'all'),checks{j}=struct('accepted',false);continue;end
            [checks{j},trajectories{j}]=ch5ForceAuditV62(x,previous,options{j},goal,o,r,k,[]);
            if checks{j}.accepted
                error=options{j}-raw;increments=diff([zeros(6,1),error],1,2);
                scores(j)=mean((error/20).^2,'all')+mean((increments/2).^2,'all');
            end
        end
        [score,choice]=min(scores);accepted(i)=isfinite(score);forceChange=nan;rateChange=nan;
        if accepted(i)
            U=options{choice};residual(:,i)=reshape(U-previous,360,1);
            [J,g]=ch5ForcePhysicsRowsV57(x,U,trajectories{choice},r,k);physical_J(i,:,:)=reshape(single(J),1,12,360);physical_g(:,i)=single(g);
            forceChange=sqrt(mean((U-raw).^2,'all'));rateChange=sqrt(mean(diff([zeros(6,1),U-raw],1,2).^2,'all'));
        end
        report.rows{end+1}=struct('selection_index',indices(i),'split',split{i},'mode',char(mode),'accepted',accepted(i),'choice',choice, ...
            'scores',scores,'selected_score',score,'checks',{checks},'force_rms_change_N',forceChange,'increment_rms_change_N',rateChange);
    end
end
report.completed=true;report.accepted=nnz(accepted);save(fullfile(directory,'repaired.mat'),'context','delta','split','indices','residual','accepted','physical_J','physical_g','report','-v7');
fid=fopen(fullfile(directory,'label_selection.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
