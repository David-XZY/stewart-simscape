function r=pgPathDiagnostic(path,task,c)
s=linspace(0,1,c.pathGrid); f=bdPathFunction(path); Q=full(f(s));
r=struct('pathHash',dpHash(path),'status','unknown','reason','insufficient_certificate', ...
    'excluded',false,'witness',[],'lowerBound',[],'gridCount',numel(s));
if any(~isfinite(Q(:))), r.reason='nonfinite_representation'; return; end
p=pgPhysics(task.model); m=task.model; [L,~,~,~,J]=p.point(Q,zeros(size(Q)),zeros(size(Q))); L=full(L); J=full(J);
r.lowerBound=pgTimeLowerBound(path,m,s,c.lengthError);
for k=1:numel(s)
    sv=svd(J(:,(k-1)*6+(1:6))*diag([1,1,1,2,2,2]));
    if any(L(:,k)<m.lmin-1e-8 | L(:,k)>m.lmax+1e-8) || min(sv)<.1-1e-8 || max(sv)/min(sv)>20+1e-8
        r.excluded=true; r.status='excluded_numeric_witness'; r.reason='length_or_singularity'; r.witness=struct('s',s(k),'length',L(:,k),'singularValues',sv); return;
    end
end
g=pgClearanceBatch(Q,task); uncertain=find(any(g<.01-1e-8,1));
for k=uncertain(:).'
    g(:,k)=pgClearance(Q(:,k),task,true).';
    for j=find(g(:,k)<.01-1e-8).'
        witness=pgDistanceWitness(Q(:,k),task,j);
        if witness.valid && witness.distanceUpper<.01-1e-8
            r.excluded=true; r.status='excluded_numeric_witness'; r.reason='payload_clearance_upper_witness';
            witness.s=s(k); witness.obstacle=j; r.witness=witness; return;
        end
    end
end
coverage=bdRobotEnvelope(task,Q,c);
if all(g(:)>=.01-1e-8) && coverage.passed, r.status='sampled_separation_pass'; r.reason='sampled_static_checks_pass'; end
r.minimumSeparationCertificate=min(g(:)); r.envelopeCertificate=coverage.minCertificate;
end
