function net = importResidualNetworkWeights(file,allowFixture)
if nargin<2, allowFixture=false; end
assert(strlength(string(file))>0 && isfile(file),'ABFD:MissingResidualWeights', ...
    'Residual-B requires an existing trained JSON asset.');
net=validateResidualNetwork(jsondecode(fileread(file)),allowFixture);
end
