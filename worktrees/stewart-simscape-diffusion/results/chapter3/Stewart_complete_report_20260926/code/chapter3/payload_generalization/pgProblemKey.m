function key=pgProblemKey(task,c)
m=rmfield(task.model,'dynamics'); options=dpOptions(c); options.ipopt.warm_start_init_point='yes';
key=dpHash(struct('robot',m,'shape',task.payload.shape,'envelope',c.envelope, ...
    'obstacles',numel(task.scene.hood.obstacles),'options',options,'intervals',c.intervals,'geometry',c.geometryVersion,'physics',c.physicsVersion));
end
