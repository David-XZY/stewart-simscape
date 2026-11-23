"""Remove unidentifiable condition weights, retaining the trained policy.

Constant training inputs have zero gradient and leave random embedding columns.
Neutralize those columns; the physical optimizer still sees full observations.
This supplies no learned moving-obstacle capability and remains development-only.
"""
from pathlib import Path
import json,shutil
import numpy as np
import scipy.io as sio
import torch
import temporal as network
from control_force import sha,write_json
from innovation_v74 import load,CH5

OUT=CH5/'artifacts/control_force_v62/raw_v76'
PRIOR=CH5/'artifacts/control_force_v62/raw_v75/seed_5101'
def main():
    OUT.mkdir(exist_ok=True,parents=True);directory=OUT/'seed_5101';directory.mkdir(exist_ok=True)
    assert not (directory/'metadata.json').exists(),'Preserve conditioning revisions.'
    d,tr,va,y,c,mu,sd,cm,cs=load();supported=c[tr].std(0)>=1e-8
    assert len(tr)==1006 and len(va)==23
    constant=c[tr][:,~supported];assert np.max(np.abs(constant-cm[~supported]))<1e-8
    state=torch.load(PRIOR/'model.pt',weights_only=False);old=network.Temporal();old.load_state_dict(state['state_dict']);old.eval()
    model=network.Temporal();model.load_state_dict(state['state_dict'])
    with torch.no_grad():model.embed[0].weight[:,:219][:,torch.tensor(~supported)]=0
    model.eval();torch.set_num_threads(2);alpha=torch.tensor(state['alpha_bar'])
    selection=sio.loadmat(CH5/'artifacts/control_force_v57/pilot/selection.mat',simplify_cells=True);ix=np.flatnonzero(selection['split']=='validation');base=selection['context'].T[ix].astype(np.float32)
    condition=torch.tensor(np.repeat((base-cm)/cs,4,axis=0));generator=torch.Generator().manual_seed(10811);noise=torch.randn(len(ix)*4,360,generator=generator)
    before=network.sample(old,condition,noise,alpha);after=network.sample(model,condition,noise,alpha);error=float((before-after).abs().max());assert error<1e-7
    state['state_dict']=model.state_dict();state['context_supported']=supported;torch.save(state,directory/'model.pt');network.export(model,state,torch.tensor((c-cm)/cs),va,directory)
    exported=sio.loadmat(directory/'weights.mat');exported={k:v for k,v in exported.items() if not k.startswith('__')};exported['parameterization']='v_feedback_innovation';exported['context_supported']=supported[:,None];sio.savemat(directory/'weights.mat',exported)
    shutil.copy2(Path(__file__),directory/'conditioning_source.py')
    old_metadata=json.loads((PRIOR/'metadata.json').read_text());write_json(directory/'metadata.json',dict(completed=True,development_only=True,prior_checkpoint_sha256=sha(PRIOR/'model.pt'),
        train_windows=len(tr),validation_windows=len(va),moving_coverage=False,unsupported_columns=(np.flatnonzero(~supported)+1).tolist(),
        constant_training_predictions_unchanged=True,fixed_96_sample_max_difference=error,matlab_sha256=sha(directory/'weights.mat'),checkpoint_sha256=sha(directory/'model.pt'),
        conditioning_source_sha256=sha(__file__),selection_rule='deterministic_zero_of_unidentifiable_condition_columns; retains_V75_checkpoint',prior_metadata=old_metadata))
    sio.savemat(OUT/'candidates_5101.mat',dict(innovation=(after.numpy()*sd+mu).T,noise=noise.numpy().T,selection_index=np.repeat(ix+1,4)[None,:],candidate=np.tile(np.arange(1,5),len(ix))[None,:],seed=5101))
    contexts=[];ids=[]
    for i,row in enumerate(base):
        o=row[42:218].reshape(8,22);active=np.flatnonzero(o[:,21]>.5)
        if not len(active):continue
        perturbed=row.copy();oi=int(active[0]);perturbed[42+22*oi+15:42+22*oi+18]=[.015,0,0];contexts.append(perturbed);ids.append(i)
    contexts=np.array(contexts,dtype=np.float32);moving=torch.tensor(np.repeat((contexts-cm)/cs,4,axis=0));fixed=condition[np.repeat(np.array(ids)*4,4)+np.tile(np.arange(4),len(ids))];n=noise[np.repeat(np.array(ids)*4,4)+np.tile(np.arange(4),len(ids))]
    old_moving=network.sample(old,moving,n,alpha);new_moving=network.sample(model,moving,n,alpha);old_base=network.sample(old,fixed,n,alpha);new_base=network.sample(model,fixed,n,alpha)
    changes_before=(old_moving-old_base).numpy()*sd;changes_after=(new_moving-new_base).numpy()*sd
    result=dict(completed=True,formal=False,contexts=len(contexts),candidates=len(contexts)*4,velocity_mps=.015,
        original_innovation_change_rms_N=float(np.sqrt(np.mean(changes_before**2))),neutralized_innovation_change_rms_N=float(np.sqrt(np.mean(changes_after**2))),
        standardized_unseen_velocity=15.,interpretation='Original response in unobserved condition dimensions comes from untrained random weights; neutralization is not moving-obstacle learning.')
    write_json(OUT/'conditioning_probe.json',result)
    sio.savemat(OUT/'conditioning_probe.mat',dict(context=contexts.T,previous=contexts[:,30:36].T,selection_index=ix[ids]+1,noise=n.numpy().T,
        original_innovation=(old_moving.numpy()*sd+mu).T,neutralized_innovation=(new_moving.numpy()*sd+mu).T))
    print(json.dumps(result));print('V76 fixed 96 samples unchanged',error,flush=True)
if __name__=='__main__':main()
