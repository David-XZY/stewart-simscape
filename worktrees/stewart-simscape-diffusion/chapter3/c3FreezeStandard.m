function standard=c3FreezeStandard(rows,c)
% Only old validation data calibrate; no new formal task is read.
selected=rows(strcmp({rows.split},'validation') & [rows.teacherPassed]);
assert(numel(selected)>=8,'C3:Calibration','Insufficient validated teacher references.');
v=vertcat(selected.teacherValues); scales=max(prctile(v,75,1),c.metricFloor);
standard=struct('schemaVersion',1,'metricNames',{c.metricNames},'groups',{c.groups}, ...
    'scales',scales,'metricFloors',c.metricFloor,'weights',c.weights,'weightNames',{c.weightNames}, ...
    'primary',1,'detourFloors',c.detourFloors,'calibrationSplit','validation', ...
    'multiplier',c.qualityMultiplier,'calibrationIds',{{selected.labelId}});
g=zeros(size(v,1),4); for k=1:4, g(:,k)=mean(v(:,c.groups{k})./scales(c.groups{k}),2); end
for name={'simple','hood'}
    mask=strcmp({selected.family},name{1}); assert(sum(mask)>=4,'C3:Calibration','Missing scene references.');
    detours=vertcat(selected.teacherDetours);
    standard.limits.(name{1})=struct('groups',c.qualityMultiplier*prctile(g(mask,:),95,1), ...
        'detours',c.qualityMultiplier*prctile(detours(mask,:),95,1), ...
        'costs',c.qualityMultiplier*prctile(g(mask,:)*c.weights.',95,1));
end
standard.selection='Primary weights selected on validation only; all three profiles must be reported.';
standard.hash=dpHash(standard);
end
