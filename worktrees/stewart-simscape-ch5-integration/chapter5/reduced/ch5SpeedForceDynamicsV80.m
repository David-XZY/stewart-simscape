function report=ch5SpeedForceDynamicsV80()
% Export the existing exact stage and its already-built A/B derivatives.
% Python training uses these native kernels offline, never in a control step.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','nonlinear_v80');
if ~isfolder(directory),mkdir(directory);end
assert(~isfile(fullfile(directory,'dynamics.json')),'ch5:ExistingAttempt','Preserve dynamics exports.');
data=load(fullfile(root,'chapter5','artifacts','control_force_v62','raw_v73','innovation_data.mat'));
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for j=1:numel(names),exists(j)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{j}));if exists(j),saved{j}=evalin('base',names{j});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
oldDirectory=pwd;cd(directory);restoreDirectory=onCleanup(@()cd(oldDirectory)); %#ok<NASGU>
count=size(data.context,2);feedback_L=zeros(6,36,count);anchor_xi=zeros(36,count);golden={};
report=struct('completed',false,'pid',feature('getpid'),'casadi_version',casadi.CasadiMeta.version(),'modes',{{}},'formal',false);
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);k=ch5ForceKernelsV64(r);n=r.plant.n;first=true;
    stageFile=sprintf('stage_%s.c',strrep(mode,'-','_'));equilibriumFile=sprintf('equilibrium_%s.c',strrep(mode,'-','_'));
    k.stage.generate(stageFile,struct('with_header',true));k.equilibrium.generate(equilibriumFile,struct('with_header',true));
    report.modes{end+1}=struct('mode',char(mode),'n',n,'stage_function',k.stage.name(),'equilibrium_function',k.equilibrium.name(), ...
        'stage_file',stageFile,'equilibrium_file',equilibriumFile);
    for index=1:count
        c=double(data.context(:,index));if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        ch5ForceTerminalV62();term=ch5ForceTerminalV62(c(1:6),r,k);
        feedback_L(:,1:n+6,index)=term.Du*term.K./term.scale';anchor_xi(1:n+6,index)=term.xi;
        if first
            x=c(1:n);previous=c(31:36);V=reshape(double(data.innovation(:,index)),6,60);
            decoder=ch5ForceInnovationV73(r);U=full(decoder(x,previous,V,feedback_L(:,1:n+6,index),term.xi));X=full(k.nodes(x,U));
            [next,A,B]=k.stage(x,U(:,1));
            golden{end+1}=struct('mode',char(mode),'index',index,'x',x,'previous',previous,'innovation',V, ...
                'L',feedback_L(:,1:n+6,index),'xi',term.xi,'U',U,'X',X,'next',full(next),'A',full(A),'B',full(B));
            first=false;
        end
    end
end
save(fullfile(directory,'anchors.mat'),'feedback_L','anchor_xi','golden','-v7');report.completed=true;
fid=fopen(fullfile(directory,'dynamics.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
