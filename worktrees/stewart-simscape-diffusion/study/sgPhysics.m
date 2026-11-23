function p=sgPhysics(model)
% Small bounded cache; every physical and geometry parameter participates.
persistent keys values;
key=dpHash(model);
if isempty(keys), keys={}; values={}; end
k=find(strcmp(keys,key),1);
if isempty(k)
    p=dpPhysics(model); keys=[keys,{key}]; values=[values,{p}];
    if numel(keys)>8, keys=keys(end-7:end); values=values(end-7:end); end
else, p=values{k}; end
end
