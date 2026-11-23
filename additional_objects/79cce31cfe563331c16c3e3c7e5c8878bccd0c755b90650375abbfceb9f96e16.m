function sgExport(runName)
% Postprocessing only. This file is excluded from the frozen numerical hash.
c=sgSetup(runName); manifest=jsondecode(fileread(fullfile(c.output,'manifest.json')));
items=manifest.cases; if iscell(items), items=[items{:}]; end
folder=fullfile(c.output,'exports'); if ~isfolder(folder), mkdir(folder); end
files=dir(fullfile(c.output,'jobs','*','result.mat'));
details={}; failureDetails={};
for k=1:numel(files)
    s=load(fullfile(files(k).folder,files(k).name)); row=s.row;
    if ~isempty(s.out)
        reasons={};
        if ~isempty(s.raw)
            names=unique({s.raw.reason});
            for j=1:numel(names), reasons{end+1}=struct('reason',names{j},'count',sum(strcmp({s.raw.reason},names{j}))); end %#ok<AGROW>
        end
        failureDetails{end+1}=struct('jobId',row.jobId,'rawFirstReasons',{reasons}, ...
            'attempts',s.out.attempts,'note','Each failed candidate reports its first rejection, not all possible violations.'); %#ok<AGROW>
    end
    item=items(strcmp({items.caseId},row.caseId));
    if isempty(item)
        probes=manifest.probes; if iscell(probes), probes=[probes{:}]; end
        item=probes(strcmp({probes.caseId},row.caseId));
    end
    task=item.truthTask;
    if ~isempty(s.out) && s.out.passed && ~isempty(s.audit)
        audit=s.audit; model=task.model;
        target=[task.q0,task.qGoal]; actual=audit.q(:,[1,end]); angles=zeros(1,2);
        for j=1:2
            R=rpy2rotmZYX(target(4:6,j)).'*rpy2rotmZYX(actual(4:6,j));
            skew=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
            angles(j)=atan2(norm(skew)/2,(trace(R)-1)/2);
        end
        stroke=(audit.L-model.lmin)./(model.lmax-model.lmin);
        details{end+1}=struct('jobId',row.jobId, ...
            'endpointPositionErrorM',max(sqrt(sum((actual(1:3,:)-target(1:3,:)).^2,1))), ...
            'endpointRotationErrorRad',max(angles),'minimumLengthUtilization',min(stroke(:)), ...
            'maximumLengthUtilization',max(stroke(:)), ...
            'auditSpeedUtilization',max(max(abs(audit.Ld)./model.actuator.ldotMax)), ...
            'auditAccelerationUtilization',max(max(abs(audit.Ldd)./model.actuator.lddotMax)), ...
            'auditRequiredForceUtilization',max(max(max(audit.FRequired./model.actuator.forceMax,audit.FRequired./model.actuator.forceMin))), ...
            'auditInterpolatedForceUtilization',max(max(max(audit.FInterpolated./model.actuator.forceMax,audit.FInterpolated./model.actuator.forceMin)))); %#ok<AGROW>
    end
    selected=strcmp(row.category,'id') && strcmp(row.mode,'known') && row.repeat==1 && strcmp(row.purpose,'main');
    selected=selected || strcmp(row.purpose,'reference');
    selected=selected || strcmp(row.purpose,'permutation') || (strcmp(row.category,'id') && strcmp(row.method,'diff_direct') && strcmp(row.purpose,'main'));
    if ~selected || isempty(s.out) || isempty(s.out.candidates), continue; end
    file=fullfile(folder,[row.jobId,'.json']); if isfile(file), continue; end
    times=linspace(0,task.T,401); count=size(s.out.candidates,3); candidates=zeros(6,101,count);
    for j=1:count
        tr=dpSpline(s.out.candidates(:,:,j),linspace(0,task.T,101),task.T); candidates(:,:,j)=tr.q;
    end
    data=struct('jobId',row.jobId,'caseId',row.caseId,'family',row.family,'method',row.method, ...
        'passed',row.passed,'repeat',row.repeat,'onlineSeconds',row.onlineSeconds,'T',task.T,'candidateQ',candidates, ...
        'obstacles',task.scene.hood.obstacles,'object',task.scene.objectCylinder, ...
        'q0',task.q0,'qGoal',task.qGoal,'trajectory',[],'limits',task.model.actuator, ...
        'lengthMin',task.model.lmin,'lengthMax',task.model.lmax);
    if ~isempty(s.out.bundle)
        tr=dpReconstruct(s.out.bundle,times); p=sgPhysics(task.model);
        [L,Ld,Ldd,F,J]=p.point(tr.q,tr.v,tr.a); J=full(J); sigma=zeros(size(times)); condition=sigma;
        for j=1:numel(times)
            sv=svd(J(:,(j-1)*6+(1:6))*diag([1,1,1,2,2,2])); sigma(j)=min(sv); condition(j)=max(sv)/min(sv);
        end
        tr.L=full(L); tr.Ld=full(Ld); tr.Ldd=full(Ldd); tr.FRequired=full(F);
        tr.sigma=sigma; tr.condition=condition;
        tr.auditTime=s.audit.time; tr.auditClearance=s.audit.clearance;
        tr.auditFRequired=s.audit.FRequired; tr.auditFInterpolated=s.audit.FInterpolated;
        data.trajectory=tr; data.quality=s.quality;
        fine=c; fine.qualityStep=c.qualityStep/2; data.qualityHalfStep=sgQuality(s.out.bundle,task,fine);
    end
    dpWriteJson(file,data);
end
fprintf('SG_EXPORT %d result files inspected\n',numel(files));
dpWriteJson(fullfile(c.output,'quality_details.json'),details);
dpWriteJson(fullfile(c.output,'failure_details.json'),failureDetails);
boundary={};
for k=1:numel(items)
    item=items(k); if ~item.generationPassed, continue; end
    for kind={'planning','truth'}
        task=item.([kind{1},'Task']); m=task.model; p=sgPhysics(m);
        for endpoint=1:2
            endpoints=[task.q0,task.qGoal]; q=endpoints(:,endpoint);
            [L,~,~,F,J]=p.point(q,zeros(6,1),zeros(6,1)); L=full(L); F=full(F);
            sv=svd(full(J)*diag([1,1,1,2,2,2])); gap=dpClearance(q,task,true);
            leg=max([0;m.lmin-L;L-m.lmax]); force=max([0;m.actuator.forceMin-F;F-m.actuator.forceMax]);
            failures={};
            if leg>1e-8, failures{end+1}='leg_length'; end
            if force>1e-6, failures{end+1}='static_force'; end
            if min(gap)<.01-1e-8, failures{end+1}='clearance_not_certified'; end
            if min(sv)<.1 || max(sv)/min(sv)>20, failures{end+1}='singularity'; end
            boundary{end+1}=struct('caseId',item.caseId,'mode',item.mode,'modelRole',kind{1}, ...
                'endpoint',endpoint,'legViolation',leg,'forceViolation',force,'forcePeak',max(abs(F)), ...
                'minClearance',min(gap),'minSigma',min(sv),'condition',max(sv)/min(sv),'failures',{failures}); %#ok<AGROW>
        end
    end
end
dpWriteJson(fullfile(c.output,'boundary_details.json'),boundary);
prefix=struct('jobId',{},'referenceJobId',{},'maximumControlDifference',{},'passed',{});
for k=1:numel(files)
    name=files(k).folder;
    if isempty(strfind(name,'__diff_direct__')) || ~isempty(strfind(name,'__k64__')), continue; end %#ok<STREMP>
    s=load(fullfile(name,files(k).name),'out','row');
    if isempty(s.out) || isempty(s.out.candidates), continue; end
    referenceId=regexprep(s.row.jobId,'__k\d+__','__k64__');
    referenceFile=fullfile(c.output,'jobs',referenceId,'result.mat');
    if ~isfile(referenceFile), continue; end
    reference=load(referenceFile,'out'); if isempty(reference.out) || isempty(reference.out.candidates), continue; end
    original=reference.out.candidates(:,:,1:size(s.out.candidates,3));
    difference=max(abs(original(:)-s.out.candidates(:)));
    prefix(end+1)=struct('jobId',s.row.jobId,'referenceJobId',referenceId, ...
        'maximumControlDifference',difference,'passed',difference<1e-10); %#ok<AGROW>
end
dpWriteJson(fullfile(c.output,'candidate_prefix_checks.json'),prefix);
end
