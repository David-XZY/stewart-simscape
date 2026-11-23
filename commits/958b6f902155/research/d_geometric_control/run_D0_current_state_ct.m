function result=run_D0_current_state_ct(task,c)
c.controllerMode='geometric-D'; c.dVariant='d0'; c.designMode='sequential';
result=runAbfdExperiment(task,c);
end
