function solution=ch5SolveProjection(optimizer,x0,parameter,lambda)
% Solver coordinates are dimensionless; all public data use SI units.
if nargin<4,lambda=[];end
args={'x0',x0./optimizer.x_scale,'p',parameter, ...
    'lbg',optimizer.lower./optimizer.g_scale,'ubg',optimizer.upper./optimizer.g_scale};
if ~isempty(lambda),args=[args,{'lam_g0',lambda}];end
raw=optimizer.solver(args{:});solution=struct('x',full(raw.x).*optimizer.x_scale,'lam_g',full(raw.lam_g),'f',full(raw.f));
% Enforce the exact affine input identity in physical coordinates. The
% optimized U is the input used by all shooting dynamics; finite NLP defects
% in its lifted increment variable must not alter the executed sequence.
H=optimizer.H;U=reshape(solution.x(6*H+(1:6*H)),6,H);previous=parameter(optimizer.n+(1:6));
exact=reshape(diff([previous,U],1,2),[],1);
solution.increment_correction_inf=norm(solution.x(1:6*H)-exact,inf);solution.x(1:6*H)=exact;
end
