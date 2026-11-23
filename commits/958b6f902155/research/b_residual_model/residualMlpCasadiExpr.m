function [mu,Sigma] = residualMlpCasadiExpr(z,net)
import casadi.*
h=(z-net.inputMean(:))./net.inputScale(:);
for k=1:numel(net.layers)
    h=net.layers(k).W*h+net.layers(k).b(:);
    if k<numel(net.layers), h=tanh(h); end
end
mu=h(1:6).*net.outputScale(:);
Sigma=diag(exp(fmin(fmax(h(7:12),-12),8)).*net.outputScale(:).^2*net.varianceCalibration);
end
