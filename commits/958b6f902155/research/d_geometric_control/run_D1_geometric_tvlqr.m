function result=run_D1_geometric_tvlqr(task,c)
c.controllerMode='geometric-D'; c.dVariant='tvlqr'; c.designMode='sequential';
result=runAbfdExperiment(task,c);
end
