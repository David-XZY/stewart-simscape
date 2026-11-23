function results=run_B2_control_benchmark(task,c)
base=c; base.modelMode='analytic'; base.designMode='sequential'; base.plannerMode='ihsid';
base.outputDir=fullfile(c.outputDir,'analytic'); results.analytic=runAbfdExperiment(task,base);
learned=base; learned.modelMode='residual-B'; learned.residualFile=c.residualFile;
learned.trajectoryFile=fullfile(base.outputDir,'reference_bundle.mat'); learned.outputDir=fullfile(c.outputDir,'residual');
results.residual=runAbfdExperiment(task,learned);
end
