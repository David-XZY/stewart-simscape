function [hessianFunction,report,gradientFunction,H]=buildAbfdCompressedHessian(x,lagrangian,sigma,lambda,progress)
% Recover the exact sparse Hessian from structurally orthogonal columns.
% A serial CasADi map reuses one Hessian-vector graph instead of expanding
% a separate second-derivative graph for every coloring direction.
import casadi.*
timer=tic;
if nargin>4, progress('building-lagrangian-gradient'); end
grad=gradient(lagrangian,x);
gradientFunction=Function('abfd_lagrangian_gradient',{x,sigma,lambda},{grad});
if nargin>4, progress('finding-hessian-sparsity'); end
pattern=gradientFunction.jac_sparsity(0,0,false); n=numel(x);
assert(isequal(size(pattern),[n,n]),'ABFD:HessianSparsity','Unexpected full Hessian dimensions.');
[row,column]=pattern.get_triplet(); row=row(:)+1; column=column(:)+1;
if isempty(row)
    H=MX.zeros(n,n); colorCount=0;
else
    if nargin>4, progress('coloring-hessian-columns'); end
    coloring=pattern.uni_coloring(pattern.T());
    [a,b]=coloring.get_triplet(); seeds=sparse(a(:)+1,b(:)+1,1,size(coloring,1),size(coloring,2));
    if size(seeds,1)~=n && size(seeds,2)==n, seeds=seeds.'; end
    assert(size(seeds,1)==n,'ABFD:HessianColoring','Coloring has an unexpected orientation.');
    colorCount=size(seeds,2); [columns,colors]=find(seeds);
    assignment=zeros(n,1); assignment(columns)=colors;
    assert(all(full(sum(seeds,2))<=1) && all(assignment(column)>0), ...
        'ABFD:HessianColoring','A nonzero Hessian column has no unique color.');
    counts=sparse(row,assignment(column),1,n,colorCount);
    assert(all(nonzeros(counts)==1),'ABFD:HessianColoring', ...
        'Each row must contain at most one Hessian entry per color for exact recovery.');
    if nargin>4, progress('building-hessian-vector-map'); end
    direction=MX.sym('hessian_direction',n,1);
    product=Function('abfd_lagrangian_hvp',{x,sigma,lambda,direction},{jtimes(grad,x,direction)});
    mapped=product.map(colorCount,'serial');
    compressed=mapped(repmat(x,1,colorCount),repmat(sigma,1,colorCount), ...
        repmat(lambda,1,colorCount),DM(seeds));
    index=row+n*(assignment(column)-1);
    H=MX(pattern,compressed(index));
end
hessianFunction=Function('abfd_compressed_hessian',{x,sigma,lambda},{H});
report=struct('method','exact sparse column coloring with serial Hessian-vector products', ...
    'variables',n,'hessianStructuralNnz',pattern.nnz(),'colors',colorCount, ...
    'buildSeconds',toc(timer),'scope','Exact AD derivatives and verified unambiguous sparse recovery; no finite-difference Hessian');
end
