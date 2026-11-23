function [callbacks,report]=buildAbfdGroupedMomentDerivatives(x,objective,constraint,meanIndices,withHessian,progress,maxDirections,roots)
% Exact AD split by dependencies: trajectory/mean variables and covariance.
% A covariance-only derivative must not repeatedly differentiate the complete
% nonlinear state rollout. This changes derivative assembly, not the NLP.
import casadi.*
if nargin<6, progress=@(~)[]; end
if nargin<7, maxDirections=1; end
validateattributes(maxDirections,{'double'},{'scalar','integer','>=',1,'<=',64});
% MX::jacobian/forward extract helper_options before constructing their
% internal Function. NLP oracle_options do not reach these explicit calls.
derivativeOptions=struct('helper_options',struct('max_num_dir',maxDirections));
timer=tic; n=numel(x); meanIndices=meanIndices(:);
assert(~isempty(meanIndices) && all(meanIndices>=1 & meanIndices<=n) && ...
    numel(unique(meanIndices))==numel(meanIndices),'ABFD:DerivativePartition','Invalid mean-variable partition.');
covarianceIndices=setdiff((1:n).',meanIndices,'stable');
assert(~isempty(covarianceIndices),'ABFD:DerivativePartition','Covariance variables are required.');
order=[meanIndices;covarianceIndices]; inverse=zeros(n,1); inverse(order)=1:n;
theta=MX.sym('trajectory_and_mean',numel(meanIndices)); eta=MX.sym('covariance_coordinates',numel(covarianceIndices));
grouped=[theta;eta]; unpack=grouped(inverse);
expressions=substitute([objective;constraint],x,unpack); f=expressions(1); g=expressions(2:end);
useRootChain=nargin>=8 && ~isempty(roots);
if useRootChain
    rootArgument=substitute(roots.arguments,x,unpack);
    affineConstraint=substitute(roots.affineConstraints,x,unpack);
    rootMap=roots.constraintMap;
    assert(isequal(size(rootMap),[numel(g),numel(rootArgument)]) && ...
        all(isfinite(nonzeros(rootMap))),'ABFD:RootCurvatureMap','Invalid root constraint map.');
    assert(~any(which_depends([f;affineConstraint;rootArgument],eta,2,true)), ...
        'ABFD:NonAffineCovariance','Root chain requires affine objective, remainder and variance in covariance coordinates.');
    % The caller constructs the full NLP from exactly this decomposition.
    % Retain it as a separately evaluable callback for independent tests.
    reconstructed=Function('abfd_root_constraint_check',{x}, ...
        {roots.affineConstraints+rootMap*sqrt(roots.arguments)});
end
progress('grouped-mean-jacobian'); Jtheta=jacobian(g,theta,derivativeOptions);
progress('grouped-covariance-jacobian'); Jeta=jacobian(g,eta,derivativeOptions);
J=[Jtheta,Jeta]; J=J(:,inverse);
J=substitute(J,grouped,x(order)); parameter=MX.sym('p',0,1);
callbacks=struct('jacobian',Function('abfd_grouped_jac_g',{x,parameter},{constraint,J}, ...
    {'x','p'},{'g','jac_g'}),'hessian',[],'fullHessian',[]);
pattern=J.sparsity();
report=struct('method','exact automatic derivatives grouped by mean and covariance dependencies', ...
    'variables',n,'meanVariables',numel(meanIndices),'covarianceVariables',numel(covarianceIndices), ...
    'jacobianStructuralNnz',pattern.nnz(),'jacobianBuildSeconds',toc(timer),'withHessian',withHessian, ...
    'maxDerivativeDirections',maxDirections, ...
    'scope','Identical full-space Jacobian and Lagrangian Hessian; no omitted coupling or finite-difference approximation');
report.rootChain=useRootChain;
if useRootChain, callbacks.reconstructedConstraints=reconstructed; end
if withHessian
    sigma=MX.sym('lam_f'); lambda=MX.sym('lam_g',numel(constraint));
    lagrangian=sigma*f+dot(lambda,g);
    progress('grouped-lagrangian-gradient'); gt=gradient(lagrangian,theta,derivativeOptions);
    progress('grouped-mean-hessian'); Htt=jacobian(gt,theta,derivativeOptions);
    % The mixed block differentiates the mean gradient with respect to eta:
    % nonlinear mean derivative calls are constants in this operation.
    progress('grouped-mixed-hessian'); Hte=jacobian(gt,eta,derivativeOptions);
    if useRootChain
        [rows,coefficients]=singleRootRows(rootMap,affineConstraint,eta);
        if all(rows>0)
            progress('grouped-root-hessian-from-constraint-jacobian');
            % A selected constraint is b(theta)+c*sqrt(a). Its existing
            % covariance Jacobian is c/(2*sqrt(a))*da/deta. Reuse it.
            Jr=Jeta(rows,:);
            weight=-(rootMap.'*lambda)./(coefficients.^2.*sqrt(rootArgument));
            Hee=Jr.'*diag(weight)*Jr;
            report.covarianceDerivativeRecovery='single-root constraint rows';
        else
            progress('grouped-root-covariance-jacobian'); B=jacobian(rootArgument,eta,derivativeOptions);
            weight=-((rootMap.'*lambda)./(4*rootArgument.^(3/2)));
            % General root maps can mix several roots into one constraint.
            Hee=B.'*diag(weight)*B;
            report.covarianceDerivativeRecovery='explicit affine-root Jacobian';
        end
        report.covarianceRootCount=numel(rootArgument);
    else
        progress('grouped-covariance-hessian'); ge=gradient(lagrangian,eta,derivativeOptions);
        Hee=jacobian(ge,eta,derivativeOptions);
    end
    H=[Htt,Hte;Hte.',Hee]; H=H(inverse,inverse);
    H=substitute(H,grouped,x(order)); pattern=H.sparsity();
    callbacks.fullHessian=Function('abfd_grouped_full_hessian',{x,sigma,lambda},{H});
    callbacks.hessian=Function('abfd_grouped_hess_lag',{x,parameter,sigma,lambda},{triu(H)}, ...
        {'x','p','lam_f','lam_g'},{'hess_gamma_x_x'});
    report.hessianStructuralNnz=pattern.nnz();
end
report.buildSeconds=toc(timer);
end
function [rows,coefficients]=singleRootRows(map,base,eta)
import casadi.*
rows=zeros(size(map,2),1); coefficients=zeros(size(rows));
rowCounts=full(sum(spones(map),2)); independent=~which_depends(base,eta,1,true);
independent=independent(:);
for k=1:numel(rows)
    [candidate,~,value]=find(map(:,k));
    selected=find(rowCounts(candidate)==1 & independent(candidate),1);
    if ~isempty(selected), rows(k)=candidate(selected); coefficients(k)=value(selected); end
end
end
