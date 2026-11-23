function report=checkAbfdInitialTube(model,scene,c)
% Fixed initial pose/covariance makes leg-length tightening independent of
% every future trajectory or gain. This is a necessary A feasibility check.
J=sgpJacobian(scene.q0,model); kin=sgpIK(scene.q0,model);
gradient=[J.Jq,zeros(6)]; variance=diag(gradient*c.initialCovariance*gradient.');
radius=c.tubeSigma*sqrt(variance+1e-16);
lower=kin.L-model.lmin-radius; upper=model.lmax-kin.L-radius;
report=struct('lengthPassed',min([lower;upper])>=-c.constraintTolerance, ...
    'minimumLengthMargin',min([lower;upper]),'lowerLengthMargins',lower,'upperLengthMargins',upper, ...
    'sigmaRadius',radius,'scope','necessary fixed-start linearized leg-length chance constraint only; no full feasibility claim');
end
