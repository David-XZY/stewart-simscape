function context=ch5Context(x,previous,goal,o,profile)
padded=zeros(30,1);padded(1:numel(x))=x;
if numel(x)==12,padded(13:30)=repmat(previous,3,1);end
context=[padded;previous;goal;o(:);double(profile=="nonideal-force")];
end
