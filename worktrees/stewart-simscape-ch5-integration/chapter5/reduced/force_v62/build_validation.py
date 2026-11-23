"""Isolated temporal pilot audit and closed-loop sources, preserve old versions."""
from pathlib import Path
import hashlib,json
BASE=Path(__file__).resolve().parents[1]
OUT=BASE.parent/'artifacts/control_force_v62'
manifest=[]
def emit(src,dst,replacements):
    text=(BASE/src).read_text(encoding='utf-8-sig')
    for old,new in replacements:
        assert old in text,(src,old)
        text=text.replace(old,new)
    (BASE/dst).write_text(text,encoding='utf-8')
    manifest.append(dict(source=src,target=dst,sha256=hashlib.sha256(text.encode()).hexdigest()))

emit('ch5SpeedForceCandidatesV60.m','ch5SpeedForceCandidatesV64.m',[
 ('ch5SpeedForceCandidatesV60','ch5SpeedForceCandidatesV64'),
 ("directory=fullfile(inputDirectory,'candidate_v60')","directory=fullfile(root,'chapter5','artifacts','control_force_v62','candidate_v64')"),
 ("data=load(fullfile(inputDirectory,'candidates_5101.mat'));w=load(fullfile(inputDirectory,'seed_5101','weights.mat'));",
  "temporal=fullfile(fileparts(directory),'temporal_v64');data=load(fullfile(temporal,'candidates_5101.mat'));w=load(fullfile(temporal,'seed_5101','weights.mat'));labels=load(fullfile(inputDirectory,'repaired.mat'),'residual','accepted');"),
 ('ch5Denoise(','ch5ForceDenoiseV64('),('ch5ForceSampleV57(','ch5ForceSampleV64('),
 ("'factor_derivatives',false","'factor_derivatives',true"),
 ('k=ch5ReducedKernelsV4(r);','k=ch5ForceKernelsV64(r);'),
 ('x=c(1:r.plant.n);previous=', 'ch5ForceTerminalV62();x=c(1:r.plant.n);previous='),
 ('ch5ForceAuditV58(', 'ch5ForceAuditV62('),('ch5ForceStepV59(', 'ch5ForceStepV63('),('ch5ForceRetractV60(', 'ch5ForceRetractV62('),
 ("'force_rms_error_N',sqrt(mean((priors(:,:,b)-rawTeacher).^2,'all')),",
  "'label_accepted',labels.accepted(index),'force_rms_error_N',sqrt(mean((priors(:,:,b)-rawTeacher).^2,'all')), ...\n                    'repaired_force_rms_error_N',sqrt(mean((priors(:,:,b)-(reshape(labels.residual(:,index),6,60)+previous)).^2,'all')), ...\n                    'repaired_increment_rms_error_N',sqrt(mean(diff([zeros(6,1),priors(:,:,b)-(reshape(labels.residual(:,index),6,60)+previous)],1,2).^2,'all')),"),
 ('V57 candidates checked','V64 temporal candidates checked')])

emit('ch5ForceControlV63.m','ch5ForceControlV64.m',[
 ('ch5ForceControlV63','ch5ForceControlV64'),('ch5ReducedKernelsV4(r)','ch5ForceKernelsV64(r)'),('ch5ForceSampleV57(','ch5ForceSampleV64('),('force-v63-development','force-v64-temporal-development')])
emit('ch5ForceV63Sfun.m','ch5ForceV64Sfun.m',[
 ('ch5ForceV63Sfun','ch5ForceV64Sfun'),('ch5ForceControlV63','ch5ForceControlV64')])
emit('ch5SpeedForceShortV63.m','ch5SpeedForceShortV64.m',[
 ('ch5SpeedForceShortV63','ch5SpeedForceShortV64'),("'short_v63'","'short_v64'"),
 ("gate=load(fullfile(fileparts(directory),'optimization_audit.mat'),'report');assert(gate.report.accepted);",
  "gate=load(fullfile(fileparts(directory),'candidate_v64','candidate_report.mat'),'report');assert(gate.report.completed&&gate.report.parity.sample_max<1e-4,'ch5:UnverifiedWeights','Require complete temporal parity and physical candidate audit.');"),
 ("'control_force_v57','pilot','seed_5101'","'control_force_v62','temporal_v64','seed_5101'"),
 ('force-v63-development','force-v64-temporal-development'),('ch5ReducedKernelsV4(r)','ch5ForceKernelsV64(r)'),
 ('ch5ForceSampleV57(','ch5ForceSampleV64('),('ch5ForceV63Sfun','ch5ForceV64Sfun'),('V63 short','V64 temporal short')])
emit('ch5SpeedForceLabelAuditV63.m','ch5SpeedForceLabelAuditV64.m',[
 ('ch5SpeedForceLabelAuditV63','ch5SpeedForceLabelAuditV64'),("'labels_v63'","'labels_v64'"),('ch5ReducedKernelsV4(r)','ch5ForceKernelsV64(r)'),('V63 labels','V64 labels')])
OUT.mkdir(exist_ok=True);(OUT/'validation_source_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print('Built temporal pilot validation entrypoints and cached dual-profile kernels.')
