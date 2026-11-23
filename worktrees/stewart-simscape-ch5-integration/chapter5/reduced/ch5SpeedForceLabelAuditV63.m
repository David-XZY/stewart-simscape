function report=ch5SpeedForceLabelAuditV63()
% Fixed 48 train / 24 validation windows, selected before new repair results.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','labels_v63');
if ~isfolder(directory),mkdir(directory);end
assert(~isfile(fullfile(directory,'repaired.mat')),'ch5:ExistingAttempt','Preserve label experiments.');
old=fullfile(root,'chapter5','artifacts','control_force_v57','pilot');s=load(fullfile(old,'selection.mat'));original=load(fullfile(old,'repaired.mat'),'residual','accepted');
indices=[unique(round(linspace(1,1024,48))),1025:1048];context=double(s.context(:,indices));split=s.split(indices);delta=double(s.delta(:,indices));
count=numel(indices);residual=nan(360,count);accepted=false(1,count);physical_J=zeros(count,12,360,'single');physical_g=zeros(12,count,'single');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'rows',{{}},'completed',false,'formal',false);timer=tic;
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);r.reduced=struct('factor_derivatives',true,'reuse_sensitivities',false,'linearization_collision_reserve',.0002);
    k=ch5ReducedKernelsV4(r);
    for i=1:count
        c=context(:,i);if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        x=c(1:r.plant.n);previous=c(31:36);goal=c(37:42);o=reshape(c(43:218),22,8);raw=previous+cumsum(reshape(delta(:,i),6,60),2);
        [U,check,traj,repair]=ch5ForceLabelRepairV63(x,previous,raw,goal,o,r,k);accepted(i)=check.accepted;
        if check.accepted,residual(:,i)=reshape(U-previous,360,1);[J,g]=ch5ForcePhysicsRowsV57(x,U,traj,r,k);physical_J(i,:,:)=reshape(single(J),1,12,360);physical_g(:,i)=single(g);end
        previousLabel=reshape(original.residual(:,indices(i)),6,60)+previous;
        report.rows{end+1}=struct('selection_index',indices(i),'split',split{i},'mode',char(mode),'source',s.source{indices(i)},'check',check,'repair',repair, ...
            'previous_label_accepted',original.accepted(indices(i)),'force_rms_change_N',sqrt(mean((U-raw).^2,'all')), ...
            'previous_force_rms_change_N',sqrt(mean((previousLabel-raw).^2,'all')), ...
            'increment_rms_change_N',sqrt(mean(diff([zeros(6,1),U-raw],1,2).^2,'all')), ...
            'previous_increment_rms_change_N',sqrt(mean(diff([zeros(6,1),previousLabel-raw],1,2).^2,'all')));
        if mod(numel(report.rows),12)==0,writeJSON(fullfile(directory,'label_audit.json'),report);fprintf('V63 labels %d/%d accepted%d\n',numel(report.rows),count,nnz(accepted));end
    end
end
report.completed=true;report.seconds=toc(timer);report.accepted=nnz(accepted);
save(fullfile(directory,'repaired.mat'),'context','split','delta','residual','accepted','physical_J','physical_g','indices','s','report','-v7');writeJSON(fullfile(directory,'label_audit.json'),report);
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
