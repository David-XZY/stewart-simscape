function key=bdProblemKey(task,c)
% Task endpoints, T and obstacle poses are NLP parameters; physics is not.
options=dpOptions(c); options.ipopt.warm_start_init_point='yes';
key=dpHash(struct('model',task.model,'object',task.scene.objectCylinder, ...
    'envelope',c.envelope,'obstacles',numel(task.scene.hood.obstacles), ...
    'options',options,'intervals',c.intervals));
end
