function [mu,Sigma] = evaluateResidualMlp(z,net)
h=(z(:)-net.inputMean(:))./net.inputScale(:);
for k=1:numel(net.layers)
    h=net.layers(k).W*h+net.layers(k).b(:);
    if k<numel(net.layers), h=tanh(h); end
end
mu=h(1:6).*net.outputScale(:);
logvar=min(max(h(7:12),-12),8);
Sigma=diag(exp(logvar).*net.outputScale(:).^2*net.varianceCalibration);
end
