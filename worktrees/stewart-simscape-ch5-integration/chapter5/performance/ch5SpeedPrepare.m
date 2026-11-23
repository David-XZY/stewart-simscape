function r=ch5SpeedPrepare(index,mode,label,polishIterations)
[c,root]=ch5Init();saved=load(fullfile(root,'chapter5','artifacts','tasks','development.mat'),'tasks');
task=saved.tasks{index};[r,preparation]=ch5PrepareCachedSimscape(task,mode,'diffusion',5101,c);
r.preparation=preparation;r.c.version=[c.version,'-speed-',label];
r.c.projection_qp_max_iterations=200;r.c.final_projection_iterations=max(1,polishIterations);
r.speed=struct('label',label,'corrections',1,'polish_iterations',polishIterations,'backtracking_steps',1,'update_seconds',.1);
setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',100000+5101+task.seed+task.index));
for name=["ch5SpeedWarm","ch5SpeedExecution","ch5SpeedIterate"]
    if isappdata(0,name),rmappdata(0,name);end
end
end
