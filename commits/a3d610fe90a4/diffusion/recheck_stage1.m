function recheck_stage1()
c=setupDiffusion(); files=dir(fullfile(c.output,'stage1','*.mat'));
for k=1:numel(files)
    file=fullfile(files(k).folder,files(k).name); s=load(file); r=s.result;
    r.report=dpValidate(r.bundle,s.task,c); r.passed=r.solverSuccess && r.maxEquality<=1e-6 && r.maxInequality<=1e-6 && r.report.passed;
    result=r; task=s.task; save([file,'.recheck.mat'],'result','task','-v7');
    fprintf('%s recheck pass=%d clearance %.9f legs %g force %g\n',files(k).name,r.passed,r.report.minClearance,r.report.maxLegViolation,r.report.maxForceViolation);
end
end
