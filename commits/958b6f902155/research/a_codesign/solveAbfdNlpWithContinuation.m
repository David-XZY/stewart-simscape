function [solution,stats,summary]=solveAbfdNlpWithContinuation(solver,bounds,x0,continuationCount,outputDirectory)
% Continue only this same immutable NLP instance after a bounded solve.
% The caller must enable IPOPT warm_start_init_point when count is positive.
% Saved iterates are diagnostics, never independently accepted trajectories.
if nargin<4, continuationCount=0; end
if nargin<5, outputDirectory=''; end
validateattributes(continuationCount,{'double'},{'scalar','integer','nonnegative','finite'});
validateattributes(x0,{'double'},{'column','real','finite'});
history=struct('segment',{},'status',{},'success',{},'iterations',{},'seconds',{}, ...
    'usedPrimalDualStart',{},'maxConstraintViolation',{});
solution=[]; previous=[];
if ~isempty(outputDirectory) && ~isfolder(outputDirectory), mkdir(outputDirectory); end
for segment=1:continuationCount+1
    if isempty(previous)
        args={'x0',x0,'lbx',bounds.lbx,'ubx',bounds.ubx,'lbg',bounds.lbg,'ubg',bounds.ubg};
    else
        args={'x0',previous.x,'lam_x0',previous.lam_x,'lam_g0',previous.lam_g, ...
            'lbx',bounds.lbx,'ubx',bounds.ubx,'lbg',bounds.lbg,'ubg',bounds.ubg};
    end
    timer=tic; solution=solver(args{:}); seconds=toc(timer); stats=solver.stats();
    current=struct('x',full(solution.x),'lam_x',full(solution.lam_x), ...
        'lam_g',full(solution.lam_g),'f',full(solution.f),'g',full(solution.g));
    assert(all(isfinite([current.x;current.lam_x;current.lam_g;current.f;current.g])), ...
        'ABFD:NonfiniteSolverContinuation','Cannot continue a nonfinite primal/dual iterate.');
    violation=max([0;bounds.lbx-current.x;current.x-bounds.ubx; ...
        bounds.lbg-current.g;current.g-bounds.ubg]);
    history(end+1)=struct('segment',segment,'status',stats.return_status, ...
        'success',logical(stats.success),'iterations',stats.iter_count,'seconds',seconds, ...
        'usedPrimalDualStart',~isempty(previous),'maxConstraintViolation',violation); %#ok<AGROW>
    summary=struct('solverCalls',segment,'continuations',segment-1, ...
        'iterations',sum([history.iterations]),'solverSeconds',sum([history.seconds]), ...
        'terminalStatus',stats.return_status,'history',history);
    if ~isempty(outputDirectory)
        checkpoint=struct('iterate',current,'stats',stats,'summary',summary,'bounds',bounds, ...
            'scope','Scaled primal/dual iterate for this immutable solver instance; no trajectory acceptance');
        save(fullfile(outputDirectory,sprintf('segment_%03d.mat',segment)),'checkpoint');
        writeAbfdJson(fullfile(outputDirectory,'status.json'),summary);
    end
    resumable=any(strcmp(stats.return_status,{'Maximum_CpuTime_Exceeded','Maximum_Iterations_Exceeded'}));
    if stats.success || ~resumable || stats.iter_count==0, break; end
    previous=current;
end
end
