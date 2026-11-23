function report=ch5SpeedForceDataV73()
% Transform admitted labels only. Task split and failed records stay intact.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','raw_v73');if ~isfolder(directory),mkdir(directory);end
assert(~isfile(fullfile(directory,'innovation_data.mat')),'ch5:ExistingAttempt','Preserve exported data.');
data=load(fullfile(root,'chapter5','artifacts','control_force_v57','pilot','repaired.mat'));
old=load(fullfile(root,'chapter5','artifacts','control_force_v62','training_geometry.mat'));
count=size(data.context,2);innovation=zeros(360,count,'single');physical_J=zeros(count,19,360,'single');physical_g=[single(data.physical_g(1:11,:));single(old.collision_g)];
terminal_J=zeros(count,36,360,'single');terminal_e=zeros(36,count,'single');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for j=1:numel(names),exists(j)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{j}));if exists(j),saved{j}=evalin('base',names{j});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'completed',false,'formal',false,'windows',nnz(data.accepted), ...
    'maximum_roundtrip_force_N',0,'maximum_linear_identity_error',0,'free_terminal_pose_derivative',true);timer=tic;
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);k=ch5ForceKernelsV64(r);decoder=ch5ForceInnovationV73(r);n=r.plant.n;
    for index=find(data.accepted)
        c=double(data.context(:,index));if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        x=c(1:n);previous=c(31:36);U=reshape(double(data.residual(:,index)),6,60)+previous;
        ch5ForceTerminalV62();anchor=ch5ForceTerminalV62(x(1:6),r,k);L=anchor.Du*anchor.K./anchor.scale';X=full(k.nodes(x,U));
        V=diff([previous,U],1,2)+L*([X(:,1:60);previous,U(:,1:59)]-anchor.xi);innovation(:,index)=single(V(:));
        reconstructed=full(decoder(x,previous,V,L,anchor.xi));roundtrip=max(abs(reconstructed-U),[],'all');
        report.maximum_roundtrip_force_N=max(report.maximum_roundtrip_force_N,roundtrip);assert(roundtrip<1e-5,'ch5:CodecParity','Round trip failed.');
        [~,aa,bb]=k.stages(X(:,1:60),U);aa=full(aa);bb=full(bb);S=zeros(n+6,360);M=zeros(360);R=[zeros(6,n),eye(6)]-L;
        for j=1:60
            rows=6*(j-1)+(1:6);M(rows,:)=R*S;M(rows,rows)=M(rows,rows)+eye(6);
            S=[aa(:,n*(j-1)+(1:n))*S(1:n,:)+bb(:,rows)*M(rows,:);M(rows,:)];
        end
        physical_J(index,:,:)=reshape(single([squeeze(data.physical_J(index,1:11,:));squeeze(old.collision_J(index,:,:))]*M),1,19,360);
        term=ch5ForceTerminalV62(X(1:6,end),r,k);P=chol(term.P,'lower');e=([X(:,end);U(:,end)]-term.xi)./term.scale;
        % Audit chooses q_s = q_H. Differentiate this choice, rather than holding q_s fixed.
        Je=(S-term.steady_jacobian*S(1:6,:))./term.scale;
        terminal_J(index,1:n+6,:)=reshape(single(P'*Je/sqrt(term.rho)),1,n+6,360);terminal_e(1:n+6,index)=single(P'*e/sqrt(term.rho));
        if mod(index,128)==0,fprintf('V73 innovation export %d/%d elapsed %.1fs\n',index,count,toc(timer));end
    end
end
context=data.context;split=data.split;accepted=data.accepted;source_data=data; %#ok<NASGU>
report.completed=true;report.seconds=toc(timer);save(fullfile(directory,'innovation_data.mat'),'innovation','context','split','accepted','physical_J','physical_g','terminal_J','terminal_e','report','-v7');
fid=fopen(fullfile(directory,'innovation_data.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
