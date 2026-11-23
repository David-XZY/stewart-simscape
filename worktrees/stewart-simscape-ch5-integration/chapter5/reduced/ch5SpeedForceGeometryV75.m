function report=ch5SpeedForceGeometryV75()
% One worst row per signed mechanical constraint: no class can be omitted.
[~,root]=ch5Init();out=fullfile(root,'chapter5','artifacts','control_force_v62','raw_v75');if ~isfolder(out),mkdir(out);end
assert(~isfile(fullfile(out,'geometry.mat')),'ch5:ExistingAttempt','Preserve geometry exports.');
data=load(fullfile(root,'chapter5','artifacts','control_force_v57','pilot','repaired.mat'));
count=size(data.context,2);physical_J=zeros(count,61,360,'single');physical_g=-ones(61,count,'single');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for j=1:numel(names),exists(j)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{j}));if exists(j),saved{j}=evalin('base',names{j});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('completed',false,'pid',feature('getpid'),'formal',false,'windows',nnz(data.accepted), ...
    'mechanical_rows',37,'command_rows',24,'all_constraint_classes_present',true,'scope','worst signed node row per class; full internal mesh checked separately');timer=tic;
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);k=ch5ForceKernelsV64(r);n=r.plant.n;H=60;
    for index=find(data.accepted)
        c=double(data.context(:,index));if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        x=c(1:n);previous=c(31:36);U=reshape(double(data.residual(:,index)),6,H)+previous;X=full(k.nodes(x,U));
        ch5ForceTerminalV62();t=ch5ForceTerminalV62(x(1:6),r,k);L=t.Du*t.K./t.scale';R=[zeros(6,n),eye(6)]-L;
        [~,aa,bb]=k.stages(X(:,1:H),U);aa=full(aa);bb=full(bb);S=zeros(n+6,360,H+1);M=zeros(360);
        for j=1:H
            rows=6*(j-1)+(1:6);M(rows,:)=R*S(:,:,j);M(rows,rows)=M(rows,rows)+eye(6);
            S(:,:,j+1)=[aa(:,n*(j-1)+(1:n))*S(1:n,:,j)+bb(:,rows)*M(rows,:);M(rows,:)];
        end
        states=reshape([X(:,1:H);X(:,2:end)],n,2*H);commands=repelem(U,1,2);[v,vx,vu]=k.mechanics(states,commands);v=full(v);vx=full(vx);vu=full(vu);
        lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2];
        hi=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf];
        scale=[r.physical.lmax-r.physical.lmin;r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;r.strict.sigmaSafe^2];hiIndex=find(isfinite(hi));
        g=[(v(hiIndex,:)-hi(hiIndex))./scale(hiIndex);(lo-v)./scale];J=zeros(61,360);g0=zeros(61,1);
        for ci=1:37
            [g0(ci),stage]=max(g(ci,:));j=ceil(stage/2);node=j+mod(stage+1,2);sgn=1;
            if ci>numel(hiIndex),mi=ci-numel(hiIndex);sgn=-1;else,mi=hiIndex(ci);end
            dx=vx(:,n*(stage-1)+(1:n));du=vu(:,6*(stage-1)+(1:6));
            J(ci,:)=sgn*(dx(mi,:)*S(1:n,:,node)+du(mi,:)*M(6*(j-1)+(1:6),:))/scale(mi);
        end
        D=diff([previous,U],1,2);Md=M-[zeros(6,360);M(1:end-6,:)];
        gCommand=[(U-r.strict.forceMax)/2000;(r.strict.forceMin-U)/2000; ...
            (D-r.strict.forceRateLimit*r.c.Ts)/160;(-D-r.strict.forceRateLimit*r.c.Ts)/160];
        for ci=1:24
            [g0(37+ci),j]=max(gCommand(ci,:));leg=mod(ci-1,6)+1;row=6*(j-1)+leg;
            if ci<=6,J(37+ci,:)=M(row,:)/2000;elseif ci<=12,J(37+ci,:)=-M(row,:)/2000;
            elseif ci<=18,J(37+ci,:)=Md(row,:)/160;else,J(37+ci,:)=-Md(row,:)/160;end
        end
        physical_J(index,:,:)=reshape(single(J),1,61,360);physical_g(:,index)=single(g0);
        if mod(index,128)==0,fprintf('V75 all-class geometry %d/%d elapsed %.1fs\n',index,count,toc(timer));end
    end
end
report.completed=true;report.seconds=toc(timer);save(fullfile(out,'geometry.mat'),'physical_J','physical_g','report','-v7');
fid=fopen(fullfile(out,'geometry.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
