"""Isolated RTI experiment: same model/gate, bounded approximate sensitivity reuse."""
from pathlib import Path
import hashlib,json
BASE=Path(__file__).resolve().parents[1];OUT=BASE.parent/'artifacts/control_force_v62';manifest=[]
def write(source,target,changes):
    text=(BASE/source).read_text(encoding='utf-8-sig')
    for a,b in changes:
        assert a in text,(source,a);text=text.replace(a,b)
    path=BASE/target;assert not path.exists(),f'Preserve {target}';path.write_text(text,encoding='utf-8')
    manifest.append(dict(source=source,target=target,sha256=hashlib.sha256(text.encode()).hexdigest()))
write('ch5ReducedJacobianV17.m','ch5ReducedJacobianV66.m',[
 ('ch5ReducedJacobianV17','ch5ReducedJacobianV66'),
 ('old=[];if isappdata',"maximumAge=.3;if isfield(r.reduced,'sensitivity_maximum_age'),maximumAge=r.reduced.sensitivity_maximum_age;end\nassert(maximumAge>=.1&&maximumAge<=.5,'ch5:SensitivityAge','Bound development sensitivity age.');\nold=[];if isappdata"),
 ('time-old.built(b)<=.10000001','time-old.built(b)<=maximumAge+1e-8'),("'maximum_age',.10000001","'maximum_age',maximumAge")])
write('ch5ForceLinearizeV62.m','ch5ForceLinearizeV66.m',[
 ('ch5ForceLinearizeV62','ch5ForceLinearizeV66'),('ch5ReducedJacobianV17','ch5ReducedJacobianV66')])
write('ch5ForceStepV63.m','ch5ForceStepV66.m',[
 ('ch5ForceStepV63','ch5ForceStepV66'),('ch5ForceLinearizeV62','ch5ForceLinearizeV66')])
start="""    [check,trajectory,cost]=ch5ForceAuditV62(x,previous,U,goal,o,r,k,qs);phases.incumbent_check=toc(timer);
    if ~check.accepted
        % Numerical terminal initializer, followed by the NEW gate; it is never an executed reference.
        [U,repair]=ch5ReducedTerminalV23(x,U,r,k,trajectory.X); %#ok<ASGLU>
        [check,trajectory,cost]=ch5ForceAuditV62(x,previous,U,goal,o,r,k,[]);
    end
    incumbent=trajectory;incumbentCheck=check;incumbentCost=cost;timer=tic;"""
replace="""    % A numerical SQP iterate needs no admission until selected for execution.
    nodes=full(k.nodes(x,U));if isempty(qs),qs=nodes(1:6,end);end
    trajectory=struct('X',nodes,'U',U,'artificial_pose',qs);
    check=struct('accepted',false,'scope','unaudited_numerical_initializer');cost=inf;
    phases.incumbent_check=toc(timer);incumbent=trajectory;timer=tic;"""
old="""    if ~check.accepted&&incumbentCheck.accepted,check=incumbentCheck;trajectory=incumbent;cost=incumbentCost;end"""
new="""    if ~check.accepted
        timer=tic;[ic,it,icost]=ch5ForceAuditV62(x,previous,incumbent.U,goal,o,r,k,incumbent.artificial_pose);
        if ~ic.accepted
            [restored,~]=ch5ReducedTerminalV23(x,incumbent.U,r,k,incumbent.X);
            [ic,it,icost]=ch5ForceAuditV62(x,previous,restored,goal,o,r,k,[]);
        end
        phases.incumbent_check=phases.incumbent_check+toc(timer);
        if ic.accepted,check=ic;trajectory=it;cost=icost;end
    end"""
write('ch5ForceControlV64.m','ch5ForceControlV66.m',[
 ('ch5ForceControlV64','ch5ForceControlV66'),('ch5ForceSampleV64','ch5ForceSampleV66'),('ch5ForceStepV63','ch5ForceStepV66'),
 ('force-v64-temporal-development','force-v66-rti-development'),(start,replace),(old,new)])
write('ch5ForceV64Sfun.m','ch5ForceV66Sfun.m',[
 ('ch5ForceV64Sfun','ch5ForceV66Sfun'),('ch5ForceControlV64','ch5ForceControlV66')])
write('ch5SpeedForceShortV64.m','ch5SpeedForceShortV66.m',[
 ('ch5SpeedForceShortV64','ch5SpeedForceShortV66'),("'short_v64'","'short_v66'"),
 ("gate=load(fullfile(fileparts(directory),'candidate_v64','candidate_report.mat'),'report');assert(gate.report.completed&&gate.report.parity.sample_max<1e-4,'ch5:UnverifiedWeights','Require complete temporal parity and physical candidate audit.');",
  "gate=jsondecode(fileread(fullfile(fileparts(directory),'inference_v66.json')));assert(gate.accepted,'ch5:BackendGate','Require full-DDIM backend parity.');"),
 ("'max_verified_candidates',2","'max_verified_candidates',1,'sensitivity_maximum_age',.3"),
 ("r.algorithm_revision='force-v64-temporal-development';","r.force_weights.checkpoint_file=fullfile(fileparts(directory),'temporal_v64','seed_5101','model.pt');r.algorithm_revision='force-v66-rti-development';"),
 ('ch5ForceSampleV64','ch5ForceSampleV66'),('ch5ForceStepV63','ch5ForceStepV66'),('ch5ForceV64Sfun','ch5ForceV66Sfun'),('V64 temporal short','V66 RTI short')])
write('ch5SpeedForceShortV66.m','ch5SpeedForceFaultV66.m',[
 ('ch5SpeedForceShortV66','ch5SpeedForceFaultV66'),("'short_v66'","'fault_v66'"),
 ('cases={1,"nonideal-force",5;4,"nonideal-force",3;1,"ideal-force",5};','cases={1,"nonideal-force",3};'),
 ("'sensitivity_maximum_age',.3","'sensitivity_maximum_age',.3,'force_rejection_window',[1,1.4]"),('V66 RTI short','V66 forced-rejection fault')])
(OUT/'rti_v66_source_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8');print('Built isolated V66 RTI and recovery-fault experiment.')
