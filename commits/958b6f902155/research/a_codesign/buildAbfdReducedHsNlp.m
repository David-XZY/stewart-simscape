function reduced=buildAbfdReducedHsNlp(original,disc,z0,mode)
% Exact elimination of affine HS equations and invertible leg-wrench balance.
% Keep the full HS motion space, every separator, and all original bounds.
% A reduced result must still pass the original NLP and dense acceptance.
import casadi.*
if nargin<4, mode='global-qr'; end
mode=validatestring(mode,{'global-qr','local-midpoint'});
assert(isfield(original,'hsReduction'),'ABFD:ReductionMetadata','Build with prepareReduction=true.');
z0=z0(:); assert(numel(z0)==numel(original.z),'ABFD:ReductionInitial');
N=disc.numIntervals; nodes=disc.numNodes; mids=disc.numMidpoints;
assert(nodes==N+1 && mids==N,'ABFD:ReductionMesh');
countX=12*(nodes-2); countMid=12*mids; countAn=6*nodes; countAm=6*mids;
motionCount=countX+countMid+countAn+countAm; forceCount=6*(nodes+mids);
linear=original.hsReduction.linearEquality;
L=jacobian(linear,original.z); assert(~depends_on(L,original.z),'ABFD:NonlinearHsReduction');
linearModel=Function('abfd_hs_linear_model',{original.z},{L,linear});
[matrixValue,value]=linearModel(z0); matrix=sparse(full(matrixValue)); offset=full(value)-matrix*z0;
assert(nnz(matrix(:,motionCount+1:end))==0 && size(matrix,1)==24*N,'ABFD:ReductionLayout');
matrix=matrix(:,1:motionCount);
if strcmp(mode,'local-midpoint')
    % Midpoint state consistency and Simpson velocity balance each solve
    % for one midpoint variable. Keep Simpson position balance explicitly,
    % so a node never becomes a function of distant intervals.
    velocityRows=12*N+reshape((7:12).'+12*(0:N-1),[],1);
    removedHs=[(1:12*N).';velocityRows];
    dependent=[countX+(1:countMid).';countX+countMid+countAn+(1:countAm).'];
    freeIndices=setdiff((1:motionCount).',dependent); freeCount=numel(freeIndices);
    pivot=matrix(removedHs,dependent); diagonal=diag(pivot);
    assert(nnz(pivot-spdiags(diagonal,0,numel(diagonal),numel(diagonal)))==0 && ...
        all(isfinite(diagonal) & abs(diagonal)>1e-12), ...
        'ABFD:LocalHsPivot','The interval-local midpoint elimination must have nonsingular diagonal pivots.');
    inverseDiagonal=spdiags(1./diagonal,0,numel(diagonal),numel(diagonal));
    transform=sparse(motionCount,freeCount); transform(freeIndices,:)=speye(freeCount);
    transform(dependent,:)=-inverseDiagonal*matrix(removedHs,freeIndices);
    base=zeros(motionCount,1); base(dependent)=-inverseDiagonal*offset(removedHs);
    reciprocalCondition=full(min(abs(diagonal))/max(abs(diagonal)));
    assert(max(sum(spones(transform),2))<=4,'ABFD:LocalHsFill','A midpoint must depend only on its neighboring node data.');
else
removedHs=(1:24*N).'; freeCount=motionCount-size(matrix,1);
triplets=cell(6,3); base=zeros(motionCount,1); freeIndices=zeros(freeCount,1);
cursor=0; reciprocalCondition=ones(6,1);
for coordinate=1:6
    row=[coordinate+12*(0:N-1);coordinate+6+12*(0:N-1)]; row=row(:);
    rows=[row;12*N+row];
    a=[coordinate+12*(0:nodes-3);coordinate+6+12*(0:nodes-3)];
    b=[coordinate+12*(0:mids-1);coordinate+6+12*(0:mids-1)];
    columns=[a(:);countX+b(:);countX+countMid+(coordinate+6*(0:nodes-1)).'; ...
        countX+countMid+countAn+(coordinate+6*(0:mids-1)).'];
    local=full(matrix(rows,columns));
    assert(nnz(matrix(rows,:))==nnz(local),'ABFD:CoupledHsCoordinates','Coordinate partition is not independent.');
    [~,R,permutation]=qr(local,'vector'); rankExpected=numel(rows);
    reciprocalCondition(coordinate)=rcond(R(:,1:rankExpected));
    assert(reciprocalCondition(coordinate)>1e-12,'ABFD:SingularHsReduction','HS pivot block is ill-conditioned.');
    dependent=permutation(1:rankExpected); free=permutation(rankExpected+1:end);
    pivot=local(:,dependent); block=-pivot\local(:,free); constant=-pivot\offset(rows);
    index=cursor+(1:numel(free)); cursor=cursor+numel(free);
    [i,j,v]=find(block);
    triplets(coordinate,:)={[columns(dependent(i));columns(free)], ...
        [index(j).';index.'],[v;ones(numel(free),1)]};
    base(columns(dependent))=constant; freeIndices(index)=columns(free);
end
assert(cursor==freeCount,'ABFD:ReductionDimension');
transform=sparse(vertcat(triplets{:,1}),vertcat(triplets{:,2}),vertcat(triplets{:,3}),motionCount,freeCount);
end
linearBasisResidual=norm(matrix(removedHs,:)*transform,inf);
linearOffsetResidual=norm(matrix(removedHs,:)*base+offset(removedHs),inf);
assert(max(linearBasisResidual,linearOffsetResidual)<1e-9,'ABFD:InexactHsReduction');
restIndices=(motionCount+forceCount+1:numel(z0)).'; keep=[freeIndices;restIndices];
eliminated=setdiff((1:numel(z0)).',keep);
y=MX.sym('reduced_hs',numel(keep)); motion=MX(base)+MX(transform)*y(1:freeCount);
partial=[motion;MX.zeros(forceCount,1);y(freeCount+1:end)];
forces=original.hsReduction.inverseForce(partial); reconstructed=[motion;forces;y(freeCount+1:end)];
% Inline the decision substitution. An opaque call to original.eval can make
% CasADi form the thousands-column full-space Jacobian before multiplying the
% reduced map, defeating the intended derivative dimension reduction.
expression=substitute([original.J;original.gEq;original.cIneq],original.z,reconstructed);
objective=expression(1); fullEquality=expression(1+(1:numel(original.gEq)));
fullInequality=expression(1+numel(original.gEq)+(1:numel(original.cIneq)));
removedEquality=[removedHs;original.hsReduction.wrenchEqualityRows(:)];
assert(numel(unique(removedEquality))==numel(removedEquality),'ABFD:ReductionEqualityLayout');
retainedEquality=setdiff((1:numel(original.gEq)).',removedEquality);
lower=eliminated(isfinite(original.lbz(eliminated))); upper=eliminated(isfinite(original.ubz(eliminated)));
inequality=[fullInequality;original.lbz(lower)-reconstructed(lower);reconstructed(upper)-original.ubz(upper)];
equality=fullEquality(retainedEquality);
reduced=struct('nlp',struct('x',y,'f',objective,'g',[equality;inequality]), ...
    'eval',Function('abfd_reduced_eval',{y},{objective,equality,inequality},struct('jac_penalty',-1)), ...
    'lift',Function('abfd_reduced_lift',{y},{reconstructed}), ...
    'fullEval',Function('abfd_reduced_full_eval',{y},{objective,fullEquality,fullInequality}), ...
    'x0',z0(keep),'lbz',original.lbz(keep),'ubz',original.ubz(keep), ...
    'lbg',[zeros(numel(equality),1);-inf(numel(inequality),1)], ...
    'ubg',zeros(numel(equality)+numel(inequality),1), ...
    'keepIndices',keep,'eliminatedIndices',eliminated,'removedEquality',removedEquality, ...
    'retainedEquality',retainedEquality,'lowerBoundIndices',lower,'upperBoundIndices',upper, ...
    'originalInequalityCount',numel(fullInequality),'motionBasis',transform,'motionOffset',base);
lifted=full(reduced.lift(reduced.x0));
reduced.report=struct('mode',mode,'fullVariables',numel(z0),'reducedVariables',numel(y), ...
    'fullMotionVariables',motionCount,'freeMotionVariables',freeCount,'eliminatedHsRank',numel(removedHs), ...
    'retainedHsEquations',24*N-numel(removedHs),'motionMapNonzeros',nnz(transform), ...
    'eliminatedForceVariables',forceCount,'reciprocalCondition',reciprocalCondition, ...
    'linearBasisResidual',linearBasisResidual,'linearOffsetResidual',linearOffsetResidual, ...
    'initialMotionCorrection',norm(lifted(1:motionCount)-z0(1:motionCount),inf), ...
    'initialForceCorrection',norm(lifted(motionCount+(1:forceCount))-z0(motionCount+(1:forceCount)),inf), ...
    'scope','Equivalent full HS motion space with exact affine/force elimination; all bounds and other constraints retained; full-space acceptance required');
end
