function report=evaluateCodesignResults(rows)
% rows must be held-out, matched records: taskId, prediction, error, force, feasible.
assert(all(strcmp({rows.split},'test')),'ABFD:HoldoutRequired','A0 validation requires held-out tasks.');
valid=[rows.feasible] & isfinite([rows.prediction]) & isfinite([rows.error]);
r=rows(valid); correlation=NaN;
if numel(r)>2
    a=averageRank([r.prediction]); b=averageRank([r.error]); matrix=corrcoef(a,b); correlation=matrix(1,2);
end
pareto=false(1,numel(rows));
for k=1:numel(rows)
    if ~rows(k).feasible, continue; end
    peers=find(strcmp({rows.taskId},rows(k).taskId) & [rows.feasible]);
    dominated=false;
    for j=peers
        dominated=dominated || (rows(j).error<=rows(k).error && rows(j).force<=rows(k).force && ...
            (rows(j).error<rows(k).error || rows(j).force<rows(k).force));
    end
    pareto(k)=~dominated;
end
report=struct('heldoutSpearman',correlation,'validCount',numel(r),'paretoMask',pareto, ...
    'caveat','Rank and Pareto membership are empirical diagnostics, not a robustness proof.');
end
function ranks=averageRank(v)
[sorted,order]=sort(v); ranks=zeros(size(v)); i=1;
while i<=numel(v)
    j=i; while j<numel(v) && sorted(j+1)==sorted(i), j=j+1; end
    ranks(order(i:j))=(i+j)/2; i=j+1;
end
end
