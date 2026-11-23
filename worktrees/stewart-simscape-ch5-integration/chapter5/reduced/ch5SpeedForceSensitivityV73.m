function report=ch5SpeedForceSensitivityV73()
% Fixed validation labels, matched random perturbations, unchanged full gate.
[~,root]=ch5Init();out=fullfile(root,'chapter5','artifacts','control_force_v62','raw_v73');if ~isfolder(out),mkdir(out);end
assert(~isfile(fullfile(out,'sensitivity.json')),'ch5:ExistingAttempt','Preserve attempts.');
data=load(fullfile(root,'chapter5','artifacts','control_force_v57','pilot','repaired.mat'));
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for j=1:numel(names),exists(j)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{j}));if exists(j),saved{j}=evalin('base',names{j});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('completed',false,'pid',feature('getpid'),'formal',false,'rows',{{}},'gain_rows',{{}},'no_projection',true);
stream=RandStream('mt19937ar','Seed',7301);timer=tic;
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);k=ch5ForceKernelsV64(r);decoder=ch5ForceInnovationV73(r);n=r.plant.n;
    indices=find(data.accepted & strcmp(data.split,'validation'));
    for index=reshape(indices,1,[])
        c=double(data.context(:,index));if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        x=c(1:n);previous=c(31:36);goal=c(37:42);o=reshape(c(43:218),22,8);U=reshape(double(data.residual(:,index)),6,60)+previous;
        ch5ForceTerminalV62();anchor=ch5ForceTerminalV62(x(1:6),r,k);L=anchor.Du*anchor.K./anchor.scale';X=full(k.nodes(x,U));
        V=diff([previous,U],1,2)+L*([X(:,1:60);previous,U(:,1:59)]-anchor.xi);
        [reconstructed,reconstructedX]=decoder(x,previous,V,L,anchor.xi);reconstructed=full(reconstructed);
        err=max(abs(reconstructed-U),[],'all');assert(err<1e-5,'ch5:CodecParity','Innovation round trip failed.');
        [~,A,B]=k.stage(anchor.x,anchor.u);A=full(A);B=full(B);Aa=[A,B;zeros(6,n),eye(6)];Ba=[B;eye(6)];
        report.gain_rows{end+1}=struct('index',index,'mode',char(mode),'roundtrip_force_max_N',err, ...
            'roundtrip_state_max',max(abs(full(reconstructedX)-X),[],'all'),'open_loop_radius',max(abs(eig(Aa))), ...
            'feedback_radius',max(abs(eig(Aa-Ba*L))));
        for amplitude=[0,.01,.1,1]
            count=3;if amplitude==0,count=1;end
            for draw=1:count
                error=randn(stream,6,60);error=amplitude*error/sqrt(mean(error.^2,'all'));
                for feedback=[false,true]
                    if feedback,trial=full(decoder(x,previous,V+error,L,anchor.xi));else,trial=U+error;end
                    [check,traj]=ch5ForceAuditV62(x,previous,trial,goal,o,r,k,[]);
                    report.rows{end+1}=struct('index',index,'mode',char(mode),'amplitude_N',amplitude,'draw',draw,'feedback',feedback, ...
                        'force_error_rms_N',sqrt(mean((trial-U).^2,'all')),'endpoint_position_error_mm',norm(traj.X(1:3,end)-X(1:3,end))*1000, ...
                        'endpoint_velocity_error_mps',norm(traj.X(7:9,end)-X(7:9,end)),'check',check);
                end
            end
        end
        fprintf('V73 raw sensitivity index %d elapsed %.1fs\n',index,toc(timer));
    end
end
report.completed=true;report.seconds=toc(timer);save(fullfile(out,'sensitivity.mat'),'report','-v7');
fid=fopen(fullfile(out,'sensitivity.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
