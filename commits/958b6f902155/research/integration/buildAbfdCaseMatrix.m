function cases=buildAbfdCaseMatrix()
% Same physical cases and feedback budget for every algorithm; fixed task times.
cases=struct('id',{},'profile',{},'overrides',{},'stateSource',{});
names={'nominal','payload','actuator','initial-error','friction-compliance','combined'};
overrides={struct(),struct('payloadMassScale',1.15,'payloadComOffset',[.01;-.01;.015],'payloadInertiaScale',1.10), ...
    struct('forceLag',.01,'forceDeadzone',5,'forceDelay',.002,'forceRate',4e4), ...
    struct('initialPoseOffset',[.001;-.001;.001;deg2rad([.1;-.1;.1])]), ...
    struct('segmentMass',1,'axialStiffness',500,'axialDamping',20,'viscousFriction',10),struct()};
for mode={'direct','external-observer'}
    for k=1:numel(names)
        profile='matched'; if strcmp(names{k},'combined'), profile='high-fidelity'; end
        cases(end+1)=struct('id',[names{k},'_',mode{1}],'profile',profile, ...
            'overrides',overrides{k},'stateSource',mode{1}); %#ok<AGROW>
    end
end
end
