function report=run_abfd_control_diagnostic(bundleFile,outputDir)
% Same reference, physical object and budget for every tested controller.
setupAbfdResearch(); data=load(bundleFile,'bundle'); b=data.bundle;
p=buildDynamicsProvider(b.model); modes={'baseline-lqi','fullstate-tvlqr','geometric-D','geometric-D'};
variants={'tvlqr','tvlqr','fixed','tvlqr'}; report=struct('id',{},'stability',{},'metrics',{});
ad=buildCasadiCompositeDynamics(p); x=[b.refs.q(:,1);b.refs.qd(:,1)];
[A,B]=ad.linearize(x,b.refs.Fleg(:,1)); A=full(A); B=full(B);
Alag=[A,B;zeros(6,12),-eye(6)/.01]; Blag=[zeros(12,6);eye(6)/.01];
Z=expm([Alag,Blag;zeros(6,24)]*.01); Znew=expm([Alag,Blag;zeros(6,24)]*.008);
Phi=Z(1:18,1:18); Bnew=Znew(1:18,19:24); Bprev=Z(1:18,19:24)-Bnew;
for k=1:numel(modes)
    id=[modes{k},'_',variants{k}]; out=fullfile(outputDir,id);
    if ~isfolder(out), mkdir(out); end
    c=makeAbfdConfig(struct('controllerMode',modes{k},'dVariant',variants{k},'outputDir',out,'plantProfile','high-fidelity'));
    r=makeAbfdRuntime(b,p,c); integral=zeros(6); leak=0;
    if strcmp(modes{k},'baseline-lqi')
        K=r.lqi.gains(:,:,1); Cx=-K(:,1:12); integral=-K(:,13:18); leak=r.lqi.leak;
    elseif strcmp(modes{k},'fullstate-tvlqr'), Cx=-r.design.schedule.K(:,:,1);
    else
        policy=buildGeometricControllerAD(p,variants{k});
        Cx=full(policy.jacobian(x,[b.refs.q(:,1);b.refs.qd(:,1);b.refs.qdd(:,1)],r.design.schedule.K(:,:,1)));
    end
    feedback=[Cx,zeros(6)]; C=[eye(6),zeros(6,12)];
    closed=[Phi+Bnew*feedback,Bprev,Bnew*integral;feedback,zeros(6),integral; ...
        .01*C,zeros(6),(1-.01*leak)*eye(6)];
    if ~strcmp(modes{k},'baseline-lqi'), closed=closed(1:24,1:24); end
    stability=struct('spectralRadius',max(abs(eig(closed))), ...
        'scope','local nominal rigid body with exact ZOH, 10 ms lag and 2 ms input delay; not a high-fidelity proof');
    writeAbfdJson(fullfile(out,'stability.json'),stability);
    % The baseline failure is already retained; repeat the geometric variants.
    metrics=struct();
    if strcmp(modes{k},'geometric-D')
        result=runAbfdSimscapeCase(b,p,c,makeHighFidelityPlantProfile('high-fidelity'));
        save(fullfile(out,'result.mat'),'result'); metrics=result.metrics;
        writeAbfdJson(fullfile(out,'metrics.json'),metrics);
    end
    report(end+1)=struct('id',id,'stability',stability,'metrics',metrics); %#ok<AGROW>
    writeAbfdJson(fullfile(outputDir,'control_diagnostic.json'),report);
end
end
