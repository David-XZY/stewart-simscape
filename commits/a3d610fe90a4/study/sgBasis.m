function [B,D,DD]=sgBasis(t,T)
% The basis depends only on the exact time grid and duration, not the model.
persistent entries;
if isempty(entries), entries={}; end
t=t(:).';
for k=1:numel(entries)
    e=entries{k};
    if e.T==T && isequal(e.t,t), B=e.B; D=e.D; DD=e.DD; return; end
end
[B,D,DD]=dpBasis(t,T);
entries=[{struct('t',t,'T',T,'B',B,'D',D,'DD',DD)},entries];
if numel(entries)>12, entries=entries(1:12); end
end
