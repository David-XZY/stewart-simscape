function [z,pending,delayed,P,lifting,joint]=applyAbfdActuatorMomentCut(z,delayed,P,residualP,lifting,index)
% Called strictly before a controller tick, after the preceding delay event.
import casadi.*
assert(isempty(lifting.predictions{index}), ...
    'ABFD:DuplicateMomentCut','Each moment cut must occur exactly once before its controller update.');
iy=[1:24,31:36]; ny=30; lower=lifting.lowerIndices;
D=lifting.stateScale; Dr=lifting.residualScale;
meanY=[z;delayed]; normalized=D\P(iy,iy)/D;
cross=D\P(iy,37:end)/Dr;
predicted=[meanY./lifting.meanScale;normalized(lower);cross(:)];
ids=(index-1)*lifting.factorSize+(1:lifting.factorSize);
cut=lifting.variables(ids);
lifting.predictions{index}=predicted;
lifting.equationBlocks{index}=predicted-cut;
lifting.knownResidualCovariance{index}=residualP;
meanY=cut(1:ny).*lifting.meanScale;
L=MX.zeros(ny,ny); L(lower)=cut(ny+(1:numel(lower)));
normalized=L+L.'-diag(diag(L));
cross=reshape(cut(ny+numel(lower)+1:end),ny,lifting.residualCount);
joint=[D*normalized*D,D*cross*Dr;Dr*cross.'*D,MX(residualP)];
P=lifting.embedding*joint*lifting.embedding.';
z=meanY(1:24); pending=meanY(25:30); delayed=pending;
end
