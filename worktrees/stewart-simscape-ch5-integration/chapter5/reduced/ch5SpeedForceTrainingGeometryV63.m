function report=ch5SpeedForceTrainingGeometryV63()
% Terminal curvature and closest obstacle rows for the fixed admitted pilot.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62');
assert(~isfile(fullfile(directory,'training_geometry.mat')),'ch5:ExistingAttempt','Preserve exported training geometry.');
file=fullfile(root,'chapter5','artifacts','control_force_v57','pilot','repaired.mat');d=load(file,'context','residual','accepted');
count=size(d.context,2);terminal_J=zeros(count,36,360,'single');terminal_e=zeros(36,count,'single');
collision_J=zeros(count,8,360,'single');collision_g=-ones(8,count,'single');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'completed',false,'windows',nnz(d.accepted),'formal',false,'scope','exact_terminal_quadratic_of_linearized_dynamics_and_node_collision_rows');timer=tic;
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);r.reduced=struct('factor_derivatives',true,'reuse_sensitivities',false);k=ch5ReducedKernelsV4(r);n=r.plant.n;H=r.c.H;
    for i=find(d.accepted)
        c=d.context(:,i);if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        ch5ForceTerminalV62();x=c(1:n);previous=c(31:36);U=reshape(d.residual(:,i),6,H)+previous;o=reshape(c(43:218),22,8);
        nodes=full(k.nodes(x,U));[~,aa,bb]=k.stages(nodes(:,1:H),U);aa=full(aa);bb=full(bb);S=zeros(n,360,H+1);
        for j=1:H,S(:,:,j+1)=aa(:,n*(j-1)+(1:n))*S(:,:,j);S(:,6*(j-1)+(1:6),j+1)=S(:,6*(j-1)+(1:6),j+1)+bb(:,6*(j-1)+(1:6));end
        term=ch5ForceTerminalV62(nodes(1:6,end),r,k);L=chol(term.P,'lower');
        e=([nodes(:,end);U(:,end)]-term.xi)./term.scale;
        Je=[S(:,:,end);zeros(6,354),eye(6)]./term.scale;
        terminal_J(i,1:n+6,:)=reshape(single(L'*Je/sqrt(term.rho)),1,n+6,360);
        terminal_e(1:n+6,i)=single(L'*e/sqrt(term.rho));
        times=(0:H)*r.c.Ts;collision=k.collision.map(H+1);
        for oi=find(o(22,:)>.5)
            ot=repmat(o(:,oi),1,H+1);ot(1:3,:)=ot(1:3,:)+o(16:18,oi)*times;
            [gaps,grad]=collision(nodes(1:6,:),ot);gaps=full(gaps);grad=full(grad);
            [gap,index]=min(gaps);collision_g(oi,i)=single((r.c.safe_distance+r.c.sample_hold_margin-gap)/.1);
            collision_J(i,oi,:)=reshape(single(-grad(:,6*(index-1)+(1:6))*S(1:6,:,index)/.1),1,1,360);
        end
        if mod(i,128)==0,fprintf('V63 training geometry window%d/%d elapsed%.1fs\n',i,count,toc(timer));end
    end
end
report.completed=true;report.seconds=toc(timer);save(fullfile(directory,'training_geometry.mat'),'terminal_J','terminal_e','collision_J','collision_g','report','-v7');
fid=fopen(fullfile(directory,'training_geometry.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
