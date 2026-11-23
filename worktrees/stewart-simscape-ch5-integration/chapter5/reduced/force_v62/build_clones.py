"""Create isolated, traceable V62 variants; never rewrite a previous attempt."""
from pathlib import Path
import hashlib
import json

REDUCED = Path(__file__).resolve().parent.parent
OUT = REDUCED.parent / 'artifacts/control_force_v62'
manifest = []


def clone(old, new, replacements):
    source = REDUCED / old
    text = source.read_text('utf-8')
    for before, after in replacements:
        assert before in text, (old, before)
        text = text.replace(before, after)
    target = REDUCED / new
    assert not target.exists(), f'Preserve {target}'
    target.write_text(text, encoding='utf-8')
    manifest.append(dict(source=old, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                         output=new, output_sha256=hashlib.sha256(target.read_bytes()).hexdigest()))


clone('ch5ForceAuditV58.m','ch5ForceAuditV62.m',[
    ('ch5ForceAuditV58','ch5ForceAuditV62'),('ch5ForceTerminalV57','ch5ForceTerminalV62'),
    ("    internal=ch5ReducedCheckRolloutV22(nodes,values,trace,U,o,r);\n    violation=max([internal.maximum_violation;reshape(r.strict.forceMin-U,[],1);reshape(U-r.strict.forceMax,[],1); ...\n        reshape(abs(D)-r.strict.forceRateLimit*r.c.Ts,[],1);0]);\n    % Receding-horizon",'    % Reuse the identical physical result already computed above.\n    % Receding-horizon')])
clone('ch5ForceLinearizeV59.m','ch5ForceLinearizeV62.m',[
    ('ch5ForceLinearizeV59','ch5ForceLinearizeV62'),
    ("'aa',aa", "'aa',aa") if "'aa',aa" in (REDUCED/'ch5ForceLinearizeV59.m').read_text() else
    ("'sensitivity',sensitivity);","'sensitivity',sensitivity,'phase_seconds',struct('nodes',nodeSeconds,'derivatives',stageSeconds,'assembly',toc(phase),'total',toc(timer)));")])
clone('ch5ForceStepV59.m','ch5ForceStepV62.m',[
    ('ch5ForceStepV59','ch5ForceStepV62'),('ch5ForceLinearizeV59','ch5ForceLinearizeV62'),('ch5ForceTerminalV57','ch5ForceTerminalV62'),
    ('for candidate=1:candidateCount','prepared=ch5ForceQpPrepareV62(linear,r);warm=[];\nfor candidate=1:candidateCount'),
    ('[delta,solve]=ch5ReducedSolveV24(linear,r);','[delta,solve,warm]=ch5ForceQpSolveV62(prepared,linear.f,warm);\n    solve.linearization_phases=linear.phase_seconds;')])
clone('ch5ForceRetractV60.m','ch5ForceRetractV62.m',[
    ('ch5ForceRetractV60','ch5ForceRetractV62'),('ch5ForceAuditV58','ch5ForceAuditV62')])
clone('ch5ForceControlV60.m','ch5ForceControlV62.m',[
    ('ch5ForceControlV60','ch5ForceControlV62'),('ch5ForceAuditV58','ch5ForceAuditV62'),
    ('ch5ForceStepV59','ch5ForceStepV62'),('ch5ForceRetractV60','ch5ForceRetractV62'),
    ("anchor=x(1:6);", "rawAudited=0;anchor=x(1:6);"),
    ('else\n    if ~isempty(saved)', 'else\n    ch5ForceTerminalV62();\n    if ~isempty(saved)'),
    ('    for b=1:4\n        [rawCheck', "    doRaw=~isfield(r.reduced,'audit_raw_candidates')||r.reduced.audit_raw_candidates;\n    if doRaw\n    rawAudited=4;\n    for b=1:4\n        [rawCheck"),
    ('    phases.trial_checks=toc(timer);timer=tic;', '    end\n    phases.trial_checks=toc(timer);timer=tic;'),
    ("'raw_checks',{rawChecks},", "'raw_audited',rawAudited,'raw_checks',{rawChecks},"),
    ('force-v60-development','force-v62-development')])
clone('ch5ForceV60Sfun.m','ch5ForceV62Sfun.m',[
    ('ch5ForceV60Sfun','ch5ForceV62Sfun'),('ch5ForceControlV60','ch5ForceControlV62')])
OUT.mkdir(parents=True,exist_ok=True)
(OUT/'clone_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print('Created',len(manifest),'isolated variants')
