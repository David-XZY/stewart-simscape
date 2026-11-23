function r=pgCollision(Q,task,mode)
if nargin<3, mode='hard'; end
g=pgClearanceBatch(Q,task); state=repmat({'unknown'},size(g)); upper=nan(size(g)); witnesses=cell(size(g));
for k=1:size(Q,2), for j=1:size(g,1)
    if g(j,k)>=.01-1e-8, state{j,k}='certified_pass';
    else
        if strcmp(mode,'hard'), gg=pgClearance(Q(:,k),task,true); g(j,k)=gg(j); end
        if g(j,k)>=.01-1e-8, state{j,k}='certified_pass'; continue; end
        w=pgDistanceWitness(Q(:,k),task,j); witnesses{j,k}=w; upper(j,k)=w.distanceUpper;
        if w.valid && w.distanceUpper<.01-1e-8, state{j,k}='witnessed_violation'; end
    end
end, end
r=struct('lower',g,'upper',upper,'status',{state},'witnesses',{witnesses},'geometryVersion',task.geometryVersion,'numericTolerance',1e-8);
end
