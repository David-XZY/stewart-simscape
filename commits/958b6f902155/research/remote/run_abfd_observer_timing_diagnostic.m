function report=run_abfd_observer_timing_diagnostic(frozenFile,outputDir)
% Training-model local analysis, not a trajectory or closed-loop acceptance.
assert(~isfolder(outputDir),'ABFD:DiagnosticExists','Use a fresh diagnostic directory.');
mkdir(outputDir); data=load(frozenFile,'tasks'); candidates=find(strcmp({data.tasks.split},'train'));
assert(numel(candidates)>=3,'ABFD:TrainingContexts','At least three frozen training contexts are required.');
rows=struct('taskId',{},'directRadius',{},'previousForceRadius',{},'currentForceRadius',{},'observerRadius',{},'covarianceIterations',{});
for j=1:3
    task=data.tasks(candidates(j)); m=task.model; p=buildDynamicsProvider(m);
    c=makeAbfdConfig(struct('controllerMode','baseline-lqi','stateSource','external-observer','sampleTime',.01));
    q=m.qHome; F=p.inverse(q,zeros(6,1),zeros(6,1));
    b=struct('publishPassed',true,'providerHash',p.hash,'model',m,'scene',task.scene, ...
        'refs',struct('q',repmat(q,1,3),'qd',zeros(6,3),'qdd',zeros(6,3), ...
        'Fleg',repmat(F,1,3),'t',[0,c.sampleTime,2*c.sampleTime]));
    r=makeAbfdRuntime(b,p,c); ad=buildCasadiCompositeDynamics(p);
    [A,B]=ad.linearize([q;zeros(6,1)],F); A=full(A); B=full(B); dt=c.sampleTime;
    Z=expm(dt*[A,B;zeros(6,18)]); Az=Z(1:12,1:12); Bz=Z(1:12,13:18);
    Ae=eye(12)+dt*A; Be=dt*B; C=[eye(6),zeros(6)]; P=1e-4*eye(12); R=1e-8*eye(6);
    for k=1:5000
        L=P*C.'/(C*P*C.'+R); H=eye(12)-L*C;
        Pc=H*P*H.'+L*R*L.'; next=Ae*Pc*Ae.'+dt*1e-6*eye(12);
        if norm(next-P,'fro')<1e-14, P=next; break; end
        P=next;
    end
    L=P*C.'/(C*P*C.'+R); G=L*C; H=eye(12)-G;
    K=r.lqi.gains(:,:,1); Kx=K(:,1:12); Ki=K(:,13:18); leak=(1-dt*r.lqi.leak)*eye(6);
    % State order: physical error, controller integral, observer prior, last force.
    used=[G,zeros(12,6),H,zeros(12,6)]; U=-Kx*used-[zeros(6,12),Ki,zeros(6,18)];
    plant=[Az,zeros(12,24)]+Bz*U;
    integral=[zeros(6,12),leak,zeros(6,18)]+dt*C*used;
    lagged=[plant;integral;Ae*used+[zeros(12,30),Be];U];
    current=[plant;integral;Ae*used+Be*U;U];
    direct=[Az-Bz*Kx,-Bz*Ki;dt*C,leak];
    rows(j)=struct('taskId',task.taskId,'directRadius',max(abs(eig(direct))), ...
        'previousForceRadius',max(abs(eig(lagged))),'currentForceRadius',max(abs(eig(current))), ...
        'observerRadius',max(abs(eig(Ae*H))),'covarianceIterations',k);
    save(fullfile(outputDir,sprintf('linearization_%02d.mat',j)),'lagged','current','direct','A','B','K','L','P','task');
end
report=struct('state','complete','rows',rows,'inputHash',abfdFileHash(frozenFile), ...
    'implementationHash',abfdFileHash([mfilename('fullpath'),'.m']), ...
    'runtimeHash',abfdFileHash(which('makeAbfdRuntime')), ...
    'scope','Three pre-training training contexts only; constant-pose ZOH plant and exact deployed LQI definition; compare two force timing assumptions for the existing Euler EKF. Does not establish actual Simscape event order or nonlinear acceptance.');
writeAbfdJson(fullfile(outputDir,'status.json'),report);
end
