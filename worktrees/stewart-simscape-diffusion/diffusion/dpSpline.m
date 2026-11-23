function tr=dpSpline(cp,t,T)
assert(isequal(size(cp),[6,16]) && all(isfinite(cp(:))),'DP:Spline','Expected finite 6x16 controls.');
[B,D,DD]=dpBasis(t,T);
tr=struct('t',t(:).','q',cp*B,'v',cp*D,'a',cp*DD,'T',T,'cp',cp,'kind','spline');
end
