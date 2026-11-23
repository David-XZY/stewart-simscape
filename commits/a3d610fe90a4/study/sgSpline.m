function tr=sgSpline(cp,t,T)
assert(isequal(size(cp),[6,16]) && all(isfinite(cp(:))),'SG:Spline','Expected finite controls.');
[B,D,DD]=sgBasis(t,T);
tr=struct('t',t(:).','q',cp*B,'v',cp*D,'a',cp*DD,'T',T,'cp',cp,'kind','spline');
end
