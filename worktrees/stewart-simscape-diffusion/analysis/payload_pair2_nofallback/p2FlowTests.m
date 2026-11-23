function report=p2FlowTests()
% Exercise the actual planner with deterministic boundary doubles, then remove them.
global P2_TEST_STATE;
folder=tempname; mkdir(folder); cleanup=onCleanup(@restore);
put('pgFinalizeTask','function t=pgFinalizeTask(t,varargin), end');
put('pgModelCheck','function pgModelCheck(varargin), end');
put('pgEncodeContext','function x=pgEncodeContext(varargin), x=zeros(99,1); end');
put('pgPhysicsSignature','function [x,s]=pgPhysicsSignature(varargin), x=zeros(255,1); s=struct(''seconds'',0,''validity'',true); end');
put('pgCandidates',sprintf(['function [pool,timing]=pgCandidates(method,task,ids,varargin)\n' ...
 'global P2_TEST_STATE; s=P2_TEST_STATE; pool={}; for id=ids, s.ids(end+1)=id; b=struct(''kind'',''spline'',''cp'',id,''T'',1);' ...
 'if s.partial&&id==2,b=[];end; pool{end+1}=struct(''bundle'',b,''meta'',struct(''failureReason'','''')); end;' ...
 's.trace{end+1}=[''generate'',mat2str(ids)];P2_TEST_STATE=s;timing=struct(''generation'',0);end']));
put('pgAccept',sprintf(['function [yes,r]=pgAccept(b,task,policy,clock,B,varargin)\n' ...
 'global P2_TEST_STATE;s=P2_TEST_STATE;repair=isfield(b,''repaired'');s.trace{end+1}=sprintf(''check:%%d:%%d'',b.cp,repair);' ...
 'if s.expire,pause(.02);end;yes=(~repair&&b.cp==s.passId)||(repair&&b.cp==s.repairPass&&s.repairs>=s.blocks);' ...
 'yes=yes&&toc(clock)<=B;P2_TEST_STATE=s;r=struct(''physicalSeconds'',0,''qualitySeconds'',0,''physicalPassed'',yes,''qualityPassed'',false,''reason'',''mock'');end']));
put('p2Repair',sprintf(['function [b,info]=p2Repair(b,varargin)\n' ...
 'global P2_TEST_STATE;s=P2_TEST_STATE;s.repairs=s.repairs+1;s.trace{end+1}=sprintf(''repair:%%d'',b.cp);P2_TEST_STATE=s;b.repaired=true;' ...
 'info=struct(''iterations'',40,''acceptedSteps'',40,''continueAllowed'',~s.stagnant,''reason'',''iteration_block'');end']));
put('pgSolve','function varargout=pgSolve(varargin), error(''P2:TestSolverForbidden'',''Unexpected solver call'');end');
addpath(folder,'-begin'); clear pgFinalizeTask pgModelCheck pgEncodeContext pgPhysicsSignature pgCandidates pgAccept p2Repair pgSolve;
task=struct('taskId','test','hash','h','modelHash','m','T',1,'endpointPassed',true,'payload',struct('shape','box','mass',100,'dimensions',[1,1,1]), ...
 'derived',struct('inertiaCOM',eye(3)),'contextSchema','test','baseSceneId','scene','split','validation');
c=struct('returnPolicy','physical','codeHash','test'); asset=struct('model',struct());
reset(); P2_TEST_STATE.passId=1; a=p2Plan(task,'ddpm_basic',1,1,asset,c);
assert(a.passed&&a.candidateGenerated==2&&a.candidateChecked==1&&P2_TEST_STATE.repairs==0); names={'pair_completed_before_check_and_early_return'};
reset(); P2_TEST_STATE.repairPass=2; a=p2Plan(task,'ddpm_signature',1,1,asset,c);
assert(a.passed&&isequal(P2_TEST_STATE.trace,{'generate[1 2]','check:1:0','check:2:0','repair:1','check:1:1','repair:2','check:2:1'})); names{end+1}='FIFO_direct_then_repair';
reset(); P2_TEST_STATE.passId=65; a=p2Plan(task,'ddpm_basic',2,1,asset,c);
assert(a.passed&&isequal(P2_TEST_STATE.ids,1:66)&&P2_TEST_STATE.repairs==64); names{end+1}='no_64_cap_and_no_regeneration';
reset(); P2_TEST_STATE.repairPass=1; P2_TEST_STATE.blocks=2; a=p2Plan(task,'regression',1,1,asset,c);
assert(a.passed&&a.candidateGenerated==1&&P2_TEST_STATE.repairs==2); names{end+1}='regression_continues_40_step_blocks';
reset(); P2_TEST_STATE.stagnant=true; a=p2Plan(task,'regression_signature',1,1,asset,c);
assert(~a.passed&&P2_TEST_STATE.repairs==1); names{end+1}='regression_stagnation_terminal';
reset(); P2_TEST_STATE.partial=true; a=p2Plan(task,'ddpm_basic',1,1,asset,c);
assert(~a.passed&&a.candidateGenerated==1&&a.candidateChecked==0); names{end+1}='partial_pair_never_accepted';
reset(); P2_TEST_STATE.passId=1; P2_TEST_STATE.expire=true; a=p2Plan(task,'ddpm_basic',.01,1,asset,c);
assert(~a.passed&&~a.returnedWithinBudget); names{end+1}='expired_acceptance_rejected';
assert(a.rankingCalls==0&&a.ihsidCalls==0); names{end+1}='no_ranking_no_fallback';
restore(); clear cleanup;
assert(c3SolverGate(true,'Solve_Succeeded',1e-6,1e-6,1e-6,true,true,true));
assert(~c3SolverGate(true,'Solve_Succeeded',0,0,1.01e-6,true,true,true));
assert(~c3SolverGate(true,'Maximum_Iterations_Exceeded',0,0,0,true,true,true));
assert(~c3SolverGate(true,'Solve_Succeeded',0,0,0,true,false,true));
assert(~c3SolverGate(true,'Solve_Succeeded',NaN,0,0,false,true,true));
names{end+1}='strict_solver_status_residual_bounds_and_deadline_gate';
report=struct('passed',true,'tests',{names});
 function reset()
  P2_TEST_STATE=struct('ids',[],'trace',{{}},'repairs',0,'passId',-1,'repairPass',-1,'blocks',1,'partial',false,'stagnant',false,'expire',false);
 end
 function put(name,code)
  f=fopen(fullfile(folder,[name,'.m']),'w'); fprintf(f,'%s',code); fclose(f);
 end
 function restore()
  if contains(path,folder), rmpath(folder); end
  clear pgFinalizeTask pgModelCheck pgEncodeContext pgPhysicsSignature pgCandidates pgAccept p2Repair pgSolve;
 end
end
