function manifest=sgManifest(c)
file=fullfile(c.output,'manifest.mat');
if isfile(file)
    s=load(file); manifest=s.manifest;
    assert(strcmp(manifest.implementationHash,sgImplementationHash(c)),'SG:Resume','Study implementation changed.'); return;
end
cases={}; ids={}; anchors={}; anchorRejections={}; families={'simple','hood'};
for f=1:2
    local={};
    for k=1:4
        seed=c.seed+100*f+k; label=sprintf('id_%s_%02d',families{f},k);
        [task,reason]=make(struct('family',families{f}),seed);
        add(task,task,label,'id','none',0,true,'known',label,reason);
        ids{end+1}=cases{end}; %#ok<AGROW>
        if ~isempty(task) && task.endpointPassed, local{end+1}=task; end %#ok<AGROW>
    end
    extra=0;
    while numel(local)<2
        extra=extra+1; assert(extra<=100,'Cannot make boundary-valid anchors.');
        [task,reason]=make(struct('family',families{f}),c.seed+100*f+10000+extra);
        if ~isempty(task) && task.endpointPassed, local{end+1}=task;
        else, anchorRejections{end+1}=struct('family',families{f},'extra',extra,'reason',reason); end %#ok<AGROW>
    end
    for k=1:2
        a=local{k}; a.groupId=sprintf('anchor_%s_%d',families{f},k); a=sgFinalizeTask(a); anchors{end+1}=a; %#ok<AGROW>
    end
end
for f=1:2
    for kind={'position','attitude'}
        if strcmp(kind{1},'position'), levels=[.08,.12]; else, levels=deg2rad([16,24]); end
        for level=1:2
            for direction=1:2
                seed=c.seed+1000*f+100*strcmp(kind{1},'attitude')+10*level+direction;
                stream=RandStream('mt19937ar','Seed',seed); d=randn(stream,3,1); d=d/max(abs(d));
                config=struct('family',families{f},'duration',7.5,'positionOffset',zeros(3,1),'attitudeOffset',zeros(3,1));
                config.([kind{1},'Offset'])=levels(level)*d;
                label=sprintf('goal_%s_%s_%d_%d',families{f},kind{1},level,direction);
                [task,reason]=make(config,seed);
                add(task,task,label,['goal_',kind{1}],kind{1},levels(level),true,'known',label,reason);
            end
        end
    end
    for variant=1:4
        label=sprintf('layout_%s_%d',families{f},variant); seed=c.seed+3000+100*f+variant;
        [task,reason]=make(struct('family',families{f},'extraObstacleCount',0),seed);
        if ~isempty(task)
            o=task.scene.hood.obstacles(end);
            if variant==1, o.center_S=o.center_S+[.1;0;0];
            elseif variant==2, o.R_S=rpy2rotmZYX([0;0;pi/4])*o.R_S; o.axes_S=o.R_S;
            elseif variant==3, o.size=1.5*o.size; o.halfSize=o.size/2;
            else
                o.name='layout_extra'; o.size=[.08;.08;.08]; o.halfSize=o.size/2;
                mid=(task.q0+task.qGoal)/2; o.center_S=mid(1:3)+rpy2rotmZYX(mid(4:6))*task.scene.objectCylinder.center_P+[.2;0;.08];
            end
            if variant==4, task.scene.hood.obstacles(end+1)=o; else, task.scene.hood.obstacles(end)=o; end
            task=sgFinalizeTask(task);
        end
        add(task,task,label,'layout','layoutVariant',variant,true,'known',label,reason);
    end
end
assert(numel(cases)==32);
for a=1:4
    for T=[3,4,6,8,10,12]
        task=anchors{a}; task.T=T; task=sgFinalizeTask(task); label=sprintf('duration_a%d_t%02d',a,T);
        add(task,task,label,'duration','duration',T,false,'known',anchors{a}.groupId,'');
    end
end
changes={}; parameter={}; values={};
for v=[50,75,125,150], param('payloadMass',v,struct('payloadMass',v)); end
for axis=1:3
    for sign=[-1,1]
        d=zeros(3,1); d(axis)=sign*.025; param(sprintf('com_%d',axis),d(axis),struct('comOffset',d));
    end
end
for v=[.5,.75,1.25,1.5], param('inertiaScale',v,struct('inertiaScale',v)); end
for v=[40,60], param('platformMass',v,struct('platformMass',v)); end
for name={'speedScale','accelerationScale','forceScale'}
    for v=[.5,.75,1.25], change=struct(); change.(name{1})=v; param(name{1},v,change); end
end
assert(numel(changes)==25);
for a=1:4
    for k=1:25
        nominal=anchors{a}; truth=nominal; truth.model=sgModel(nominal.model,nominal.scene,changes{k}); truth=sgFinalizeTask(truth);
        label=sprintf('physical_a%d_p%02d',a,k);
        add(truth,truth,label,'physical',parameter{k},values{k},false,'known',nominal.groupId,'');
        add(nominal,truth,[label,'_unknown'],'physical',parameter{k},values{k},false,'unknown',nominal.groupId,'');
    end
end
for f=1:2
    for k=1:8
        seed=c.seed+5000+100*f+k; stream=RandStream('mt19937ar','Seed',seed);
        config=struct('family',families{f},'positionOffset',.16*(rand(stream,3,1)-.5), ...
            'attitudeOffset',deg2rad(32)*(rand(stream,3,1)-.5),'duration',[4,8,12]);
        config.duration=config.duration(1+mod(k-1,3)); [nominal,reason]=make(config,seed); truth=nominal;
        if ~isempty(truth)
            d=zeros(3,1); d(1+mod(k-1,3))=.025*(-1)^k;
            mass=75+50*mod(k,2); cap=.75+.5*mod(f+k,2);
            truth.model=sgModel(nominal.model,nominal.scene,struct('payloadMass',mass,'comOffset',d, ...
                'forceScale',cap,'speedScale',.75+.5*mod(k,2))); truth=sgFinalizeTask(truth);
        end
        label=sprintf('joint_%s_%02d',families{f},k);
        add(truth,truth,label,'joint','joint',k,false,'known',label,reason);
        add(nominal,truth,[label,'_unknown'],'joint','joint',k,false,'unknown',label,reason);
    end
end
assert(sum(cellfun(@(x)strcmp(x.mode,'known'),cases))==172);
assert(sum(cellfun(@(x)strcmp(x.mode,'unknown'),cases))==116);
probes={};
for k=1:numel(ids)
    original=ids{k}; if isempty(original.planningTask), continue; end
    task=original.planningTask; task.scene.hood.obstacles=task.scene.hood.obstacles(end:-1:1); task=sgFinalizeTask(task);
    probe=original; probe.caseId=[original.caseId,'_permuted']; probe.category='permutation'; probe.core=false;
    probe.planningTask=task; probe.truthTask=task; probe.parentCaseId=original.caseId; probes{end+1}=probe; %#ok<AGROW>
end
manifest=struct('schemaVersion',1,'kind','frozen-model-generalization','seed',c.seed, ...
    'cases',{cases},'probes',{probes},'anchors',{anchors},'anchorRejections',{anchorRejections}, ...
    'implementationHash',sgImplementationHash(c),'modelHash',dpHash(fileread(c.modelFile)), ...
    'parameterChanges',{changes},'scope','Fixed payload exterior and mechanism geometry; no retraining.');
manifest.hash=dpHash(manifest); save(file,'manifest','-v7'); dpWriteJson(fullfile(c.output,'manifest.json'),manifest);
fprintf('SG_MANIFEST known=172 unknown=116 core=32 probes=%d\n',numel(probes));
    function [task,reason]=make(config,seed)
        task=[]; reason='';
        try, task=makeTask(config,seed,'study_test',false); task=sgFinalizeTask(task);
        catch e, task=[]; reason=[e.identifier,': ',e.message]; end
    end
    function add(planning,truth,label,category,name,value,core,mode,group,reason)
        family=''; if ~isempty(planning), family=planning.family;
        elseif ~isempty(strfind(label,'simple')), family='simple'; elseif ~isempty(strfind(label,'hood')), family='hood'; end %#ok<STREMP>
        cases{end+1}=struct('caseId',label,'category',category,'parameter',name,'value',value, ...
            'core',core,'mode',mode,'family',family,'groupId',group,'parentCaseId','', ...
            'planningTask',planning,'truthTask',truth,'generationPassed',~isempty(planning), ...
            'generationReason',reason);
    end
    function param(name,value,change)
        parameter{end+1}=name; values{end+1}=value; changes{end+1}=change;
    end
end
