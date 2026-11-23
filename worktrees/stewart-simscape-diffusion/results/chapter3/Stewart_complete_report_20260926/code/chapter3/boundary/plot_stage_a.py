"""Plot actual Stage-A geometry probes, without claiming trajectory feasibility."""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parents[2]
folder = root / 'results/chapter3/boundary_stageA_v2'
task = json.loads((folder/'task.json').read_text(encoding='utf-8-sig'))
data = json.loads((folder/'stageA.json').read_text(encoding='utf-8-sig'))
sys.path.insert(0, str(root/'study'))
from plot_trajectories import box, rotations

fig = plt.figure(figsize=(13, 7))
ax3 = fig.add_subplot(221, projection='3d')
ax2 = fig.add_subplot(222)
gap_ax = fig.add_subplot(223)
yaw_ax = fig.add_subplot(224)
obj = task['scene']['objectCylinder']
axis = np.asarray(obj['axis_P'], dtype=float)
axis /= np.linalg.norm(axis)
side = np.cross(axis, np.eye(3)[np.argmin(abs(axis))])
side /= np.linalg.norm(side)
up = np.cross(axis, side)
for obstacle in task['scene']['hood']['obstacles']:
    box(ax3, obstacle)
    center = np.asarray(obstacle['center_S'])
    half = np.asarray(obstacle['halfSize'])
    ax2.add_patch(plt.Rectangle((center[:2]-half[:2])*1000, 2000*half[0], 2000*half[1], color='grey', alpha=.25))
    ax2.text(center[0]*1000,center[1]*1000,obstacle['name'].replace('fixture_',''),ha='center',fontsize=8)
theta = np.linspace(0, 2*np.pi, 49)
for pose, label, color in [(task['q0'],'Start','#1565a5'),(task['qGoal'],'Goal','#b45816')]:
    q = np.asarray(pose)
    R = rotations(q[3:,None,None])[0,0]
    center = q[:3]+R@np.asarray(obj['center_P'])
    rings = []
    for sign in [-1,1]:
        points = center[:,None]+R@(.5*sign*obj['length']*axis[:,None]+obj['radius']*(side[:,None]*np.cos(theta)+up[:,None]*np.sin(theta)))
        rings.append(points*1000)
        ax3.plot(*rings[-1],color=color,lw=.8)
        ax2.plot(*rings[-1][:2],color=color,lw=.8)
    for k in [0,12,24,36]:
        joined = np.column_stack([rings[0][:,k],rings[1][:,k]])
        ax3.plot(*joined,color=color,lw=.8)
        ax2.plot(*joined[:2],color=color,lw=.8)
    ax3.scatter(*q[:3]*1000,color=color,label=label)
for probe,color in zip(data['probes'],['#1565a5','#b45816']):
    q=np.asarray(probe['q']); s=np.asarray(probe['s'])
    label='Straight pose probe' if probe['name'].startswith('straight') else '20 deg yaw probe'
    ax3.plot(*(q[:3]*1000),color=color,ls='--')
    gap_ax.plot(s,np.min(probe['clearanceLowerBounds'],axis=0)*1000,color=color,label=label)
    yaw_ax.plot(q[1]*1000,np.rad2deg(q[5]),color=color,label=label)
gap_ax.axhline(10,color='black',ls='--',lw=.8)
gap_ax.set(xlabel='Path progress s',ylabel='Minimum separation certificate / mm',title='Below 10 mm: not accepted; not an infeasibility proof')
gap_ax.legend(fontsize=8)
yaw_ax.set(xlabel='Platform y / mm',ylabel='Yaw / deg',title='Two hand-built geometry probes, no dynamics solve')
ax2.set(xlabel='x / mm',ylabel='y / mm',title='Top view of the actual endpoint geometries')
ax2.set_aspect('equal',adjustable='box'); ax2.set_xlim(-900,900); ax2.set_ylim(-400,400)
ax3.set(xlabel='x / mm',ylabel='y / mm',zlabel='z / mm',title='Stage A: two fixtures and full payload cylinders')
ax3.set_xlim(-900,900); ax3.set_ylim(-500,500); ax3.set_zlim(950,1450); ax3.legend(fontsize=8)
fig.suptitle('Development prototype: endpoints passed; full trajectory feasibility and coupling not established',fontsize=11)
fig.tight_layout()
fig.savefig(folder/'stageA_geometry.png',dpi=150)
fig.savefig(folder/'stageA_geometry.svg')
print(folder/'stageA_geometry.png')
