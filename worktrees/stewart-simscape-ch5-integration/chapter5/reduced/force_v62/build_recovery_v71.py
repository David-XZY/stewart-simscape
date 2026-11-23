"""Versioned development controller; retain all V68/V70 failure evidence."""
from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[3];RED=ROOT/'chapter5/reduced';OUT=ROOT/'chapter5/artifacts/control_force_v62'
def clone(source,target,replacements):
    destination=RED/target
    if destination.exists():raise FileExistsError(destination)
    text=(RED/source).read_text(encoding='utf-8-sig')
    for old,new in replacements:
        if old not in text:raise ValueError(f'Missing source in {source}: {old[:80]}')
        text=text.replace(old,new)
    destination.write_text(text,encoding='utf-8')
    return {'source':source,'target':target,'sha256':hashlib.sha256(destination.read_bytes()).hexdigest()}
def main():
    start="    elseif ~isempty(saved)&&isfield(saved,'numerical_trajectory')&&~isempty(saved.numerical_trajectory)\n        shift=min(r.c.H,max(0,round((time-saved.numerical_time)/r.c.Ts)));old=saved.numerical_trajectory;\n        U=[old.U(:,shift+1:end),repmat(old.U(:,end),1,shift)];qs=old.artificial_pose;\n"
    files=[]
    files.append(clone('ch5ForceControlV70.m','ch5ForceControlV71.m',[
        ('ch5ForceControlV70','ch5ForceControlV71'),
        (start,"    % Rejected numerical iterates cannot track the fallback input history.\n    % Rebuild from the actual augmented state; admission still uses full gates.\n"),
        ('force-v70-terminal-execution-development','force-v71-recovery-reset-development')]))
    files.append(clone('ch5ForceV70Sfun.m','ch5ForceV71Sfun.m',[
        ('ch5ForceV70Sfun','ch5ForceV71Sfun'),('ch5ForceControlV70','ch5ForceControlV71')]))
    gate="gate=jsondecode(fileread(fullfile(fileparts(directory),'recovery_audit_v71.json')));assert(gate.completed,'ch5:RecoveryGate','Require frozen failed-state audit.');\nnew=gate.rows([gate.rows.reset_to_current_brake]);old=gate.rows(~[gate.rows.reset_to_current_brake]);\nassert(sum(arrayfun(@(z)z.initializer_check.accepted,new))>sum(arrayfun(@(z)z.initializer_check.accepted,old)),'ch5:RecoveryGate','Reset must improve actual-state initialization.');"
    for kind in ['Fault','Full']:
        source=f'ch5SpeedForce{kind}V68.m';target=f'ch5SpeedForce{kind}V71.m'
        text=(RED/source).read_text(encoding='utf-8-sig')
        if kind=='Fault':
            original="gate=jsondecode(fileread(fullfile(fileparts(directory),'inference_v65.json')));assert(gate.completed&&gate.accepted,'ch5:InferenceGate','Require single precision MATLAB parity.');"
        else:
            original="gate=jsondecode(fileread(fullfile(fileparts(directory),'short_v68','summary.json')));assert(gate.completed&&all(arrayfun(@(v)v.metrics.all_constraints_valid,gate.rows)),'ch5:ShortGate','Require short Simscape physical checks.');"
        if original not in text:
            original=next(line for line in text.splitlines() if line.startswith('gate='))
        full_gate=gate
        if kind=='Full':full_gate+="\nfault=jsondecode(fileread(fullfile(fileparts(directory),'fault_v71','summary.json')));\nassert(fault.completed&&all(arrayfun(@(z)z.metrics.all_constraints_valid,fault.rows)),'ch5:RecoveryFaultGate','Require complete fault-injection physical check.');"
        replacements=[(f'ch5SpeedForce{kind}V68',f'ch5SpeedForce{kind}V71'),
            (f"'{kind.lower()}_v68'",f"'{kind.lower()}_v71'"),('ch5ForceV68Sfun','ch5ForceV71Sfun'),
            ('force-v68-rti-matlab-single-development','force-v71-recovery-reset-development'),(original,full_gate)]
        if kind=='Full':
            replacements.append(('cases={1,"nonideal-force",15;4,"nonideal-force",15};',
                                 'cases={1,"nonideal-force",15;4,"nonideal-force",15;1,"nonideal-force",15;1,"ideal-force",15};'))
            line='        index=cases{ci,1};mode=cases{ci,2};stop=cases{ci,3};r=ch5ReducedPrepareV48(index,mode);'
            extra="""
        if ci==3
            targetTasks=load(fullfile(root,'chapter5','artifacts','tasks','development.mat'),'tasks');
            r.task.goals=[r.task.goals(:,1),targetTasks.tasks{3}.goals(:,end)];r.task.goal_times=[0,8];
            r.task.family=3;r.task.subtype='terminal_goal_switch';r.task.id='development_terminal_switch_070';
            r.task.seed=70070;r.task.index=703;index=3;
        end"""
            replacements.append((line,line+extra))
        files.append(clone(source,target,replacements))
    (OUT/'recovery_source_manifest_v71.json').write_text(json.dumps(files,indent=2),encoding='utf-8')
    print('Created',len(files),'versioned recovery sources')
if __name__=='__main__':main()
