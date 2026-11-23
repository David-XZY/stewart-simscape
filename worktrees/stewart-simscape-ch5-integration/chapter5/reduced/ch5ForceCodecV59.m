function [z,U,D]=ch5ForceCodecV59(value,previous,w,operation)
% An invertible coordinate change, not a new physical decision variable.
if strcmp(operation,'encode')
    D=reshape(double(value),6,60,[]);U=cumsum(D,2)+previous;
    residual=reshape(U-previous,360,[]);z=(residual-double(w.force_mean(:)))./double(w.force_std(:));
elseif strcmp(operation,'decode')
    z=double(value);residual=double(w.force_mean(:))+double(w.force_std(:)).*z;
    U=reshape(residual,6,60,[])+previous;
    D=diff(cat(2,repmat(previous,1,1,size(U,3)),U),1,2);
else,error('ch5:ForceCodecOperation','Unknown coordinate operation.');end
end
