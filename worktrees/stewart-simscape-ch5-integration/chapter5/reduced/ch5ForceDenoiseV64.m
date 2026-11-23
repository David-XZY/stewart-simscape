function v=ch5ForceDenoiseV64(z,c,t,w)
% CPU-only temporal CNN inference; exported cross-correlation matrices.
assert(strcmp(w.architecture,'temporal_cnn_v64'),'ch5:TemporalWeights','Require temporal model.');
B=size(z,2);H=60;frequency=exp(-log(10000)*(0:15)'/15);phase=frequency*t;
if size(c,2)==1&&B>1,c=repmat(c,1,B);end
if size(phase,2)==1&&B>1,phase=repmat(phase,1,B);end
condition=[c;sin(phase);cos(phase)];
for i=1:2,condition=tanh(double(w.(sprintf('embedW%d',i)))*condition+double(w.(sprintf('embedb%d',i))));end
j=linspace(0,1,H);position=repmat([j;sin(pi*j);cos(pi*j)],1,1,B);
value=tanh(convolution(cat(1,reshape(z,6,H,B),position),double(w.inputW),double(w.inputb),1,3));
width=size(value,1);
for i=1:numel(w.dilations)
    film=double(w.(sprintf('filmW%d',i)))*condition+double(w.(sprintf('filmb%d',i)));
    scale=reshape(1+.1*tanh(film(1:width,:)),width,1,B);bias=reshape(film(width+1:end,:),width,1,B);
    first=convolution(value,double(w.(sprintf('block%dc1W',i))),double(w.(sprintf('block%dc1b',i))),double(w.dilations(i)),3);
    second=convolution(tanh(first.*scale+bias),double(w.(sprintf('block%dc2W',i))),double(w.(sprintf('block%dc2b',i))),double(w.dilations(i)),3);
    value=value+.1*tanh(second);
end
v=reshape(convolution(value,double(w.outputW),double(w.outputb),1,1),360,B);
end
function y=convolution(x,W,b,dilation,kernel)
C=size(x,1);H=size(x,2);B=size(x,3);patch=zeros(C*kernel,H*B);
for j=1:kernel
    offset=(j-(kernel+1)/2)*dilation;source=(1:H)+offset;valid=source>=1&source<=H;
    shifted=zeros(C,H,B);shifted(:,valid,:)=x(:,source(valid),:);
    patch(C*(j-1)+(1:C),:)=reshape(shifted,C,H*B);
end
y=reshape(W*patch+b,size(W,1),H,B);
end
