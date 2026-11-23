function [cp,report]=dpFitSpline(result,task,c)
t=linspace(0,task.T,301); tr=dpReconstruct(result.bundle,t); [B,~,~]=dpBasis(t,task.T);
cp=dpBaseline(task); cp(:,4:13)=0; cp(:,4:13)=(tr.q-cp*B)/B(4:13,:);
report=dpValidate(struct('kind','spline','cp',cp,'T',task.T),task,c);
if ~report.passed
    rawReport=report;
    [projected,projectionReport,projectionInfo]=dpQuickFit(task,cp,c);
    if projectionReport.passed
        cp=projected; report=projectionReport; report.quickFit=projectionInfo; report.rawFitPassed=false;
        return;
    end
    [cp,fitInfo]=dpOptimizeSpline(task,cp,c);
    report=dpValidate(struct('kind','spline','cp',cp,'T',task.T),task,c);
    report.constrainedFit=fitInfo; report.quickFit=projectionInfo; report.rawFitPassed=rawReport.passed;
    report.passed=report.passed && fitInfo.success;
end
end
