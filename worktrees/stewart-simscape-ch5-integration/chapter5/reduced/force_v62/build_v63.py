"""Isolated online budget/recovery variant, following the exact V62 audit."""
from pathlib import Path
import hashlib,json
REDUCED=Path(__file__).resolve().parent.parent
manifest=[]
def clone(old,new,replacements):
    src=REDUCED/old;text=src.read_text('utf-8')
    for before,after in replacements:
        assert before in text,(old,before);text=text.replace(before,after)
    dest=REDUCED/new;assert not dest.exists();dest.write_text(text,encoding='utf-8')
    manifest.append(dict(source=old,source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),output=new,output_sha256=hashlib.sha256(dest.read_bytes()).hexdigest()))
clone('ch5ForceStepV62.m','ch5ForceStepV63.m',[
 ('ch5ForceStepV62','ch5ForceStepV63'),
 ('baseF=linear.f;',"priorJ=sqrt(.05*r.c.Ts)*[M/20,zeros(360,6)];priorRes=sqrt(.05*r.c.Ts)*(U(:)-firstPrior(:))/20;\nQtask=linear.Q-2*(priorJ'*priorJ);ftask=linear.f-2*priorJ'*priorRes;taskBase=sum(res.^2)-sum(priorRes.^2);\nbaseF=linear.f;"),
 ('if solve.finite\n        proposed',"if solve.finite\n        infos{candidate}.predicted_task_cost=taskBase+ftask'*delta+.5*delta'*Qtask*delta;\n        proposed")])
clone('ch5ForceControlV62.m','ch5ForceControlV63.m',[
 ('ch5ForceControlV62','ch5ForceControlV63'),('ch5ForceStepV62','ch5ForceStepV63'),
 ('refresh=isempty(saved)||isempty(saved.trajectory)||time-saved.refresh_time',
  "eventChanged=false;\nif ~isempty(saved)\n    dt=time-saved.observation_time;predictedCenter=saved.observed(1:3,:)+saved.observed(16:18,:)*dt;\n    active=(o(22,:)>.5)|(saved.observed(22,:)>.5);\n    eventChanged=any(abs(o(4:15,active)-saved.observed(4:15,active))>1e-10,'all')|| ...\n        any(abs(o(16:18,active)-saved.observed(16:18,active))>1e-4,'all')|| ...\n        any(abs(o(1:3,active)-predictedCenter(:,active))>.001,'all');\nend\nrefresh=isempty(saved)||eventChanged||time-saved.refresh_time"),
 ('if ~refresh\n    shift=',"if ~refresh&&isempty(saved.trajectory)\n    [u,d]=ch5ReducedRecoveryFast(x,previous,o,r,k,time,anchor);\n    prediction=struct('refresh',false,'chosen',0,'candidates',{{}},'checks',{{}},'raw_checks',{{}},'raw_audited',0,'raw_accepted',0, ...\n        'projected_accepted',0,'denoise_attempts',{{}},'final_attempts',{{}},'restoration_attempts',{{}},'goal',goal,'observed',o, ...\n        'cost',inf,'phases',phases,'full_horizon_checked',false,'validation_scope','recovery_executed_interval', ...\n        'full_horizon_failure_active',true,'recovery_active',true,'terminal_certified',false,'algorithm_revision','force-v63-development');\n    saved.goal=goal;saved.observed=o;saved.observation_time=time;setappdata(0,'ch5ReducedExecution',saved);return;\nend\nif ~refresh\n    shift="),
 ('    else\n        brake=ch5ReducedBrakeSequence(r);U=full(brake(x,previous));qs=[];',
  "    elseif ~isempty(saved)&&isfield(saved,'numerical_trajectory')&&~isempty(saved.numerical_trajectory)\n        shift=min(r.c.H,max(0,round((time-saved.numerical_time)/r.c.Ts)));old=saved.numerical_trajectory;\n        U=[old.U(:,shift+1:end),repmat(old.U(:,end),1,shift)];qs=old.artificial_pose;\n    else\n        brake=ch5ReducedBrakeSequence(r);U=full(brake(x,previous));qs=[];"),
 ('    for b=1:4\n        attempt=solves{b};',
  "    ranking=inf(1,4);for b=1:4,if isfield(solves{b},'predicted_task_cost'),ranking(b)=solves{b}.predicted_task_cost;end;end\n    [~,order]=sort(ranking);limit=4;if isfield(r.reduced,'max_verified_candidates'),limit=r.reduced.max_verified_candidates;end\n    verified=0;\n    for b=order\n        attempt=solves{b};\n        if verified>=limit,attempt.retained=false;attempt.trials={};attempt.audit_skipped_by_budget=true;attempts{b}=attempt;continue;end\n        verified=verified+1;"),
 ('        attempts{b}=attempt;\n    end',
  "        attempts{b}=attempt;\n    end"),
 ('saved.goal=goal;saved.observed=o;setappdata',
  "if refresh,saved.refresh_time=time;saved.numerical_time=time;saved.numerical_trajectory=trajectory;end\nsaved.goal=goal;saved.observed=o;saved.observation_time=time;setappdata"),
 ('force-v62-development','force-v63-development')])
clone('ch5ForceV62Sfun.m','ch5ForceV63Sfun.m',[
 ('ch5ForceV62Sfun','ch5ForceV63Sfun'),('ch5ForceControlV62','ch5ForceControlV63')])
out=REDUCED.parent/'artifacts/control_force_v62'
(out/'clone_v63_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print('Created',len(manifest),'V63 variants')
