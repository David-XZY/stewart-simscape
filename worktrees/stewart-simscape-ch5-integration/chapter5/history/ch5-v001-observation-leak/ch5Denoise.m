function noise=ch5Denoise(action,context,t,w)
frequency=exp(-log(10000)*(0:15)'/15);phase=frequency*t;
if size(context,2)==1&&size(action,2)>1,context=repmat(context,1,size(action,2));end
if size(phase,2)==1&&size(action,2)>1,phase=repmat(phase,1,size(action,2));end
value=[action;context;sin(phase);cos(phase)];
for i=1:3,value=tanh(double(w.(sprintf('W%d',i)))*value+double(w.(sprintf('b%d',i))));end
noise=double(w.W4)*value+double(w.b4);
if isfield(w,'residual_noise_skip')&&w.residual_noise_skip,noise=noise+action;end
end
