function manifest=c3Manifest(c)
% Tasks are frozen before ANY formal planner runs; construction uses endpoints only.
file=fullfile(c.output,'manifest.mat'); if isfile(file), z=load(file); manifest=z.manifest; return; end
cases={}; rejected={}; anchors={}; families={'simple','hood'};
categories={'id','goal_position','goal_attitude','layout','joint'};
for f=1:2
    for g=1:5
        for k=1:4
            seed=c.seed+f*10000+g*100+k; config=struct('family',families{f});
            stream=RandStream('mt19937ar','Seed',seed); direction=randn(stream,3,1); direction=direction/max(abs(direction));
            if g==2, config.positionOffset=(.06+.02*k)*direction; end
            if g==3, config.attitudeOffset=deg2rad(12+3*k)*direction; end
            if g==4, config.extraObstacleCount=0; end
            if g==5, config.positionOffset=.08*direction; config.attitudeOffset=deg2rad(16)*direction; config.duration=4+2*k; end
            task=makeTask(config,seed,'chapter3_test',false);
            if g==4
                % Explicit endpoint-safe added obstacle. No generated path is consulted.
                o=task.scene.hood.obstacles(1); o.name='ch3_added'; o.size=[.08;.08;.08]; o.halfSize=o.size/2;
                o.R_S=eye(3); o.axes_S=o.R_S; placed=false;
                mid=.5*(task.q0+task.qGoal); center=mid(1:3)+rpy2rotmZYX(mid(4:6))*task.scene.objectCylinder.center_P;
                for distance=[.14,.20,.28,.40,.60]
                    for angle=linspace(0,2*pi,13)
                        o.center_S=center+distance*[cos(angle);sin(angle);.25]; o.center_B=o.center_S;
                        probe=task; probe.scene.hood.obstacles=o;
                        if min(dpClearance(task.q0,probe,true))>.012 && min(dpClearance(task.qGoal,probe,true))>.012
                            placed=true; break;
                        end
                    end
                    if placed, break; end
                end
                assert(placed,'C3:Obstacle','Could not place endpoint-safe obstacle.');
                task.scene.hood.obstacles(end+1)=o;
            end
            if g==5, task.model=sgModel(task.model,task.scene,struct('payloadMass',75+50*mod(k,2),'speedScale',.75+.5*mod(k,2))); end
            task=sgFinalizeTask(task); id=sprintf('core_%s_%s_%02d',families{f},categories{g},k);
            add(task,id,categories{g},id,'',0,'',0);
            if g==1 && task.endpointPassed && sum(cellfun(@(a)strcmp(a.family,families{f}),anchors))<2
                task.anchorCaseId=id; anchors{end+1}=task; %#ok<AGROW>
            end
        end
    end
    attempt=0;
    while sum(cellfun(@(a)strcmp(a.family,families{f}),anchors))<2
        attempt=attempt+1; assert(attempt<=100); task=makeTask(families{f},c.seed+90000+f*100+attempt,'chapter3_anchor',false);
        if task.endpointPassed, task.anchorCaseId=task.taskId; anchors{end+1}=task; else, rejected{end+1}=task.taskId; end %#ok<AGROW>
    end
end
changes={}; names={}; values={};
for v=[50,75,125,150], param('payloadMass',v,struct('payloadMass',v)); end
for axis=1:3
    for sign=[-1,1], d=zeros(3,1); d(axis)=sign*.025; param(sprintf('com_%d',axis),d(axis),struct('comOffset',d)); end
end
for v=[.5,.75,1.25,1.5], param('inertiaScale',v,struct('inertiaScale',v)); end
for v=[40,60], param('platformMass',v,struct('platformMass',v)); end
for name={'speedScale','accelerationScale','forceScale'}
    for v=[.5,.75,1.25], change=struct(); change.(name{1})=v; param(name{1},v,change); end
end
for a=1:4
    group=sprintf('anchor_%d',a);
    for T=[3,4,6,8,10,12]
        task=anchors{a}; task.T=T; task=sgFinalizeTask(task);
        add(task,sprintf('scan_a%d_T%d',a,T),'duration',group,'duration',T,'',0);
    end
    for k=1:numel(changes)
        task=anchors{a}; task.model=sgModel(task.model,task.scene,changes{k}); task=sgFinalizeTask(task);
        add(task,sprintf('scan_a%d_p%02d',a,k),'physical',group,names{k},values{k},'',0);
    end
end
for f=1:2
    anchor=anchors{find(cellfun(@(a)strcmp(a.family,families{f}),anchors),1)};
    for kind={'position','attitude','duration','payload'}
        sequence=[families{f},'_',kind{1}];
        for step=0:10
            task=anchor;
            if strcmp(kind{1},'position'), task.qGoal(1)=task.qGoal(1)+.004*step;
            elseif strcmp(kind{1},'attitude'), task.qGoal(6)=task.qGoal(6)+deg2rad(.8*step);
            elseif strcmp(kind{1},'duration'), task.T=anchor.T+.2*step;
            else, task.model=sgModel(task.model,task.scene,struct('payloadMass',100+5*step)); end
            % Endpoint pose changes move the target assembly consistently; fixed obstacles stay fixed.
            if any(strcmp(kind{1},{'position','attitude'}))
                R=rpy2rotmZYX(task.qGoal(4:6))*rpy2rotmZYX(anchor.qGoal(4:6)).';
                task.scene.box.R_S=rpy2rotmZYX(task.qGoal(4:6)); task.scene.box.rpy=task.qGoal(4:6);
                task.scene.box.center_S=task.qGoal(1:3)-task.scene.box.R_S*(task.scene.phase.pCylinderGoal_B-task.scene.objectCylinder.center_P);
                [task.scene.qWaypoint,~,task.scene.target]=computeCylinderWaypointAndGoal(task.scene);
                for j=1:numel(task.scene.hood.obstacles)
                    o=task.scene.hood.obstacles(j);
                    if strcmp(task.family,'hood') && any(strcmp(o.name,{'roof','leftSkirt','rightSkirt','target-body'}))
                        o.center_S=task.qGoal(1:3)+R*(o.center_S-anchor.qGoal(1:3)); o.R_S=R*o.R_S; o.axes_S=o.R_S;
                        task.scene.hood.obstacles(j)=o;
                    end
                end
            end
            task=sgFinalizeTask(task);
            add(task,sprintf('sequence_%s_%02d',sequence,step),'sequence',sequence,kind{1},step,sequence,step);
        end
    end
end
assert(numel(cases)==252 && numel(changes)==25);
manifest=struct('schemaVersion',1,'seed',c.seed,'cases',{cases},'anchors',{anchors},'anchorRejections',{rejected}, ...
    'scope','40 independent, 124 dependent parameter scans, 88 sequential conditions; new tasks, known parameters only.');
manifest.hash=dpHash(manifest); save(file,'manifest','-v7'); dpWriteJson(fullfile(c.output,'manifest.json'),manifest);
    function add(task,id,category,group,parameter,value,sequence,step)
        task.taskId=id; task.groupId=group; task=sgFinalizeTask(task);
        rootGroup=group; if isfield(task,'anchorCaseId'), rootGroup=task.anchorCaseId; end
        cases{end+1}=struct('caseId',id,'category',category,'groupId',group,'family',task.family, ...
            'rootGroupId',rootGroup,'parameter',parameter,'value',value,'sequence',sequence,'step',step,'task',task);
    end
    function param(name,value,change)
        names{end+1}=name; values{end+1}=value; changes{end+1}=change;
    end
end
