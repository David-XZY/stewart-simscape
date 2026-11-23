function tr=sgReconstruct(bundle,t)
if strcmp(bundle.kind,'spline'), tr=sgSpline(bundle.cp,t,bundle.T);
else, tr=dpReconstruct(bundle,t); end
end
