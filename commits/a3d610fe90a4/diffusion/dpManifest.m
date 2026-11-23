function manifest=dpManifest(kind,c)
validatestring(kind,{'pilot','formal'}); n=6; if strcmp(kind,'formal'), n=50; end
tasks={}; rejected=struct('family',{},'seed',{},'reason',{});
for family={'simple','hood'}
    for k=1:n
        if strcmp(kind,'formal')
            if k<=35, split='train'; elseif k<=40, split='validation'; else, split='test'; end
            seed=20262000+k;
        else
            if k<=4, split='train'; elseif k==5, split='validation'; else, split='test'; end
            seed=20261000+k;
        end
        % Deterministic bounded rejection BEFORE freezing, no path-success filtering.
        accepted=false;
        for retry=0:99
            trialSeed=seed+retry*10000;
            try
                task=makeTask(family{1},trialSeed,split,false);
                assert(task.endpointPassed,'DP:Boundary','%s',task.endpointReason);
                tasks{end+1}=task; accepted=true; break; %#ok<AGROW>
            catch e
                rejected(end+1)=struct('family',family{1},'seed',trialSeed,'reason',e.message); %#ok<AGROW>
            end
        end
        assert(accepted,'DP:Manifest','Endpoint rejection budget exhausted.');
    end
end
manifest=struct('schemaVersion',1,'kind',kind,'tasks',{tasks},'rejected',rejected, ...
    'implementationHash',dpImplementationHash(c),'config',dpRunConfig(c),'seed',c.seed,'splitUnit','whole geometric task');
manifest.hash=dpHash(manifest);
end
