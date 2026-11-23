"""Render reproducible figures from MATLAB-exported representative tasks."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def box(ax, obstacle):
    center=np.asarray(obstacle['center_S']).reshape(3)
    half=np.asarray(obstacle['halfSize']).reshape(3)
    rotation=np.asarray(obstacle['R_S'])
    vertices=np.array([[i,j,k] for i in [-1,1] for j in [-1,1] for k in [-1,1]])*half
    vertices=(vertices@rotation.T+center)*1000
    faces=[[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]]
    ax.add_collection3d(Poly3DCollection([vertices[f] for f in faces],alpha=.12,facecolor='gray',edgecolor='.5',linewidth=.4))


def cylinder(ax, pose, obj, color):
    r,p,y=pose[3:]; cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    rotation=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                       [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    center=np.asarray(obj['center_P']).reshape(3)
    theta=np.linspace(0,2*np.pi,33)
    rings=[]
    for end in [-.5,.5]:
        local=np.vstack([np.full(theta.size,end*obj['length']),obj['radius']*np.cos(theta),obj['radius']*np.sin(theta)])
        world=(rotation@(local+center[:,None])+pose[:3,None])*1000; rings.append(world)
        ax.plot(*world,color=color,alpha=.3,lw=.7)
    for k in [0,8,16,24]:
        ax.plot(*np.column_stack([rings[0][:,k],rings[1][:,k]]),color=color,alpha=.3,lw=.7)


def rotations(euler):
    """ZYX matrices for arrays shaped (roll/pitch/yaw, time, candidate)."""
    r,p,y=euler; cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    matrix=np.empty(r.shape+(3,3))
    matrix[...,0,0]=cy*cp; matrix[...,0,1]=cy*sp*sr-sy*cr; matrix[...,0,2]=cy*sp*cr+sy*sr
    matrix[...,1,0]=sy*cp; matrix[...,1,1]=sy*sp*sr+cy*cr; matrix[...,1,2]=sy*sp*cr-cy*sr
    matrix[...,2,0]=-sp; matrix[...,2,1]=cp*sr; matrix[...,2,2]=cp*cr
    return matrix


def permutation_changes(records, output):
    lookup={d['jobId']:d for d in records}; rows=[]
    for record in records:
        if not record['caseId'].endswith('_permuted'): continue
        original=lookup.get(record['jobId'].replace('_permuted__','__'))
        if original is None: continue
        a,b=np.asarray(original['candidateQ']),np.asarray(record['candidateQ'])
        assert a.shape==b.shape
        position=np.sqrt(np.mean(np.sum((a[:3]-b[:3])**2,axis=0),axis=0))
        relative=np.einsum('...ji,...jk->...ik',rotations(a[3:]),rotations(b[3:]))
        skew=np.stack([relative[...,2,1]-relative[...,1,2],relative[...,0,2]-relative[...,2,0],
                       relative[...,1,0]-relative[...,0,1]],axis=-1)
        angle=np.arctan2(np.linalg.norm(skew,axis=-1)/2,(np.trace(relative,axis1=-2,axis2=-1)-1)/2)
        orientation=np.sqrt(np.mean(angle**2,axis=0))
        rows.append(dict(jobId=record['jobId'],originalJobId=original['jobId'],
                         meanPairedPositionRmsM=float(position.mean()),maxPairedPositionRmsM=float(position.max()),
                         meanPairedRotationRmsRad=float(orientation.mean()),maxPairedRotationRmsRad=float(orientation.max()),
                         comparison='Identical candidate IDs and noise; 101 normalized-time samples; SO(3) rotation difference.'))
    if rows:
        with (output/'obstacle_order_candidate_changes.csv').open('w',encoding='utf-8-sig',newline='') as stream:
            writer=csv.DictWriter(stream,list(rows[0])); writer.writeheader(); writer.writerows(rows)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('directory',type=Path); args=parser.parse_args()
    root=args.directory; output=root/'report'/'trajectories'; output.mkdir(parents=True,exist_ok=True)
    records=[json.loads(f.read_text(encoding='utf-8-sig')) for f in sorted((root/'exports').glob('*.json'))]
    permutation_changes(records,output)
    convergence=[]
    for record in records:
        for key,value in record.get('quality',{}).items():
            fine=record.get('qualityHalfStep',{}).get(key)
            if key in ['seconds','gridStep'] or not isinstance(value,(int,float)) or not isinstance(fine,(int,float)): continue
            convergence.append(dict(jobId=record['jobId'],metric=key,step5ms=value,step2_5ms=fine,
                                    relativeDifference=abs(value-fine)/max(abs(fine),1e-12)))
    if convergence:
        with (output/'quality_grid_convergence.csv').open('w',encoding='utf-8-sig',newline='') as stream:
            writer=csv.DictWriter(stream,list(convergence[0]));writer.writeheader();writer.writerows(convergence)
    selected=[]
    # First lexical paired domain task in each scene, deterministic and explicit.
    for family in ['simple','hood']:
        options=sorted(set(d['caseId'] for d in records if d['family']==family and d['caseId'].startswith('id_')))
        for case in options:
            group={d['method']:d for d in records if d['caseId']==case and d['passed'] and d['repeat']==1}
            if 'geo_ihsid' in group and 'diff_fast' in group:
                selected.append((case,group)); break
    for case,group in selected:
        fast=group['diff_fast']; base=group['geo_ihsid']; candidate=np.asarray(fast['candidateQ'])
        fig=plt.figure(figsize=(14,6)); ax=fig.add_subplot(121,projection='3d'); zoom=fig.add_subplot(122,projection='3d')
        for panel in [ax,zoom]:
            for k in range(candidate.shape[2]): panel.plot(*[candidate[i,:,k]*1000 for i in range(3)],color='#77a9d5',alpha=.25,lw=.8)
            for method,d in group.items():
                if method not in ['geo_ihsid','diff_ihsid','diff_fast','retrieve_fast']: continue
                q=np.asarray(d['trajectory']['q']); panel.plot(*[q[i]*1000 for i in range(3)],lw=2,label=method)
            panel.scatter(*np.asarray(fast['q0'])[:3]*1000,color='green',s=40)
            panel.scatter(*np.asarray(fast['qGoal'])[:3]*1000,color='red',s=40)
            panel.set(xlabel='x (mm)',ylabel='y (mm)',zlabel='z (mm)')
        obstacles=fast['obstacles']; obstacles=obstacles if isinstance(obstacles,list) else [obstacles]
        for obstacle in obstacles: box(ax,obstacle)
        cylinder(ax,np.asarray(fast['q0']),fast['object'],'green'); cylinder(ax,np.asarray(fast['qGoal']),fast['object'],'red')
        ax.set_title(case+' | payload at start and goal'); zoom.set_title('Platform path detail: 64 raw candidates + accepted outputs')
        zoom.legend(fontsize=8); fig.tight_layout(); fig.savefig(output/(case+'_paths.png'),dpi=160); plt.close(fig)
        fig,axes=plt.subplots(3,2,figsize=(11,8))
        for method,d in group.items():
            tr=d['trajectory']; values=np.asarray(tr['q'])
            for i in range(3):
                axes[i,0].plot(tr['t'],values[i]*1000,label=method,lw=1)
                axes[i,1].plot(tr['t'],np.rad2deg(values[i+3]),label=method,lw=1)
        for i in range(3):
            axes[i,0].set_ylabel(['x (mm)','y (mm)','z (mm)'][i]); axes[i,1].set_ylabel(['roll (deg)','pitch (deg)','yaw (deg)'][i])
        for panel in axes.flat: panel.grid(alpha=.2); panel.set_xlabel('Time (s)')
        axes[0,0].legend(fontsize=8); fig.suptitle(case+' | pose display; rotation quality uses SO(3) metrics')
        fig.tight_layout(); fig.savefig(output/(case+'_pose.png'),dpi=160); plt.close(fig)
        fig,axes=plt.subplots(3,2,figsize=(12,10))
        for method,d in [('geo_ihsid',base),('diff_fast',fast)]:
            tr=d['trajectory']; t=np.asarray(tr['t']); color='tab:orange' if method=='geo_ihsid' else 'tab:blue'
            gap=np.asarray(tr['auditClearance']); gap=gap.reshape(1,-1) if gap.ndim==1 else gap
            axes[0,0].plot(tr['auditTime'],gap.min(axis=0)*1000,label=method,color=color)
            for index,key in enumerate(['L','Ld','Ldd','FRequired'],start=1):
                ax=axes.flat[index]; values=np.asarray(tr[key]);
                ax.plot(t,values.T,color=color,alpha=.6,lw=.8)
            axes[2,1].plot(t,tr['sigma'],color=color,label=method)
        axes[0,0].axhline(10,color='r',ls='--',lw=.8); axes[0,0].legend()
        titles=['Minimum clearance (mm)','Leg length (m)','Leg speed (m/s)','Leg acceleration (m/s²)','Required force (N)','Minimum scaled singular value']
        for ax,title in zip(axes.flat,titles): ax.set_title(title); ax.set_xlabel('Time (s)'); ax.grid(alpha=.2)
        axes[2,1].axhline(.1,color='r',ls='--',lw=.8)
        axes[0,1].axhline(min(np.asarray(fast['lengthMin']).reshape(-1)),color='r',ls='--',lw=.6)
        axes[0,1].axhline(max(np.asarray(fast['lengthMax']).reshape(-1)),color='r',ls='--',lw=.6)
        limits=fast['limits']
        for ax,key in [(axes[1,0],'ldotMax'),(axes[1,1],'lddotMax'),(axes[2,0],'forceMax')]:
            limit=max(np.asarray(limits[key]).reshape(-1)); ax.axhline(limit,color='r',ls='--',lw=.6); ax.axhline(-limit,color='r',ls='--',lw=.6)
        fig.suptitle(case+' | full audit retained in MAT; plotted dynamics sampled at 401 times')
        fig.tight_layout(); fig.savefig(output/(case+'_constraints.png'),dpi=160); plt.close(fig)
    (output/'selection.json').write_text(json.dumps([case for case,_ in selected],indent=2),encoding='utf-8')
    print('Representative paired tasks:',[case for case,_ in selected])


if __name__=='__main__': main()
