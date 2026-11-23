function report=testCertifiedEqualityProjection(outputDir)
% Certificate tampering, nonlinear rows and unbounded residuals must fail.
import casadi.*
assert(~isfolder(outputDir),'ABFD:ProjectionTestExists','Use a fresh fixture directory.'); mkdir(outputDir);
x=MX.sym('x',3); equality=x(1)+x(2)-1;
g=[equality;2*equality+1e-15;MX(0);x(3)^2-4;x(1)-10];
oracle=Function('abfd_projection_fixture',{x},{dot(x,x),g});
file=fullfile(outputDir,'oracle.casadi'); oracle.save(file);
kit=struct('oracleHash',abfdFileHash(file),'c',struct('solverTolerance',1e-9), ...
    'bounds',struct('lbx',-3*ones(3,1),'ubx',3*ones(3,1),'lbg',[zeros(4,1);-inf],'ubg',zeros(5,1)));
point=struct('x',[.4;.6;2],'lam_g',[.7;-.2;3;1;0]); checkpointHash=abfdHash(point);
details=struct('affineRows',[1;2;3],'keptRows',1,'dependentRows',[2;3], ...
    'matrix',sparse([1,1,0;2,2,0;0,0,0]),'offset',[-1;-2+1e-15;0],'coefficients',[2;0]);
audit=struct('state','complete','passed',true,'withinRoundoffBudget',true,'oracleHash',kit.oracleHash, ...
    'checkpointHash',checkpointHash,'candidateDependentRows',[2;3]);
folder=writeCase('valid',details,kit,audit);
projection=verifyAbfdEqualityProjection(oracle,kit,point,folder,checkpointHash);
assert(isequal(projection.keepOriginalRows,[1;4;5]) && ...
    norm(projection.mappedMultipliers-[.3;0;0;1;0],inf)<1e-14 && ...
    projection.report.boundedDependenceResidual<1e-14,'ABFD:EqualityProjectionContract', ...
    'Keep the nonlinear constraint and inequality and transfer all equality multiplier contributions.');
bad=details; bad.coefficients(1)=2.01;
reject('wrong-coefficient',bad,kit,audit,'ABFD:EqualityDependencyBound');
bad=details; bad.affineRows=[1;2;4]; bad.dependentRows=[2;4]; other=audit; other.candidateDependentRows=[2;4];
reject('nonlinear-row',bad,kit,other,'ABFD:EqualityProjectionNonlinear');
unbounded=kit; unbounded.bounds.lbx(1)=-inf; unbounded.bounds.ubx(1)=inf;
bad=details; bad.coefficients(1)=2+1e-14;
reject('unbounded-small-error',bad,unbounded,audit,'ABFD:EqualityDependencyBound');
folder=writeCase('exact-unbounded',details,unbounded,audit);
verified=verifyAbfdEqualityProjection(oracle,unbounded,point,folder,checkpointHash);
assert(verified.report.boundedDependenceResidual<1e-14,'ABFD:ExactUnboundedProjection', ...
    'An exactly zero coefficient residual does not depend on unbounded coordinates.');
other=audit; other.oracleHash='different-original-problem';
reject('wrong-oracle',details,kit,other,'ABFD:EqualityProjectionSource');
report=struct('passed',true,'checks',6,'mappedMultiplierError', ...
    norm(projection.mappedMultipliers-[.3;0;0;1;0],inf), ...
    'boundedDependenceResidual',projection.report.boundedDependenceResidual, ...
    'scope','Synthetic equality representation and refusal contracts; no Stewart trajectory, training or optimization acceptance');
writeAbfdJson(fullfile(outputDir,'report.json'),report);
    function folder=writeCase(name,details,kit,audit)
        folder=fullfile(outputDir,name); mkdir(folder);
        save(fullfile(folder,'affine_equality_audit.mat'),'details','kit');
        writeAbfdJson(fullfile(folder,'status.json'),audit);
    end
    function reject(name,details,kit,audit,identifier)
        folder=writeCase(name,details,kit,audit); caught=false;
        try
            verifyAbfdEqualityProjection(oracle,kit,point,folder,checkpointHash);
        catch exception
            caught=strcmp(exception.identifier,identifier);
        end
        assert(caught,'ABFD:EqualityProjectionRefusal','Expected refusal %s for %s.',identifier,name);
    end
end
