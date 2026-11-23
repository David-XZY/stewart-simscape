function test_diffusion_solver_failures()
% Exercise real solver failure handling with a deliberately zero iteration budget.
c=setupDiffusion(); c.maxIter=0; c.output=fullfile(c.output,'finite_solver_failure_test');
task=makeTask('simple',20260915,'development',true);
candidates=repmat(dpBaseline(task),1,1,8);
out=refineAndValidate(task,candidates,c);
assert(~out.passed && isempty(out.best) && numel(out.attempts)==8);
for k=1:8
    s=load(out.attempts(k).file,'result');
    assert(~isempty(s.result) && ~s.result.solverSuccess && ~s.result.passed);
    assert(~out.attempts(k).passed && ~isempty(out.attempts(k).reason));
end
dpWriteJson(fullfile(c.output,'report.json'),struct('passed',true,'finiteCandidates',true, ...
    'solverFailurePreserved',true,'attempts',out.attempts));
fprintf('DP_SOLVER_FAILURE_TESTS_PASS\n');
end
