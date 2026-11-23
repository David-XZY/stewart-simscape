"""Render actual bounded development evidence; never fabricate model comparisons."""
from pathlib import Path
import json
import math
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from run_boundary import RESULTS,read,write


def main():
    output=RESULTS/'boundary_delivery_v1'; output.mkdir(parents=True,exist_ok=True)
    screen=read(RESULTS/'boundary_mechanism_screen_v1/summary.json')
    probe=read(RESULTS/'boundary_geometric_probe_v1/summary.json')
    trajectory=read(RESULTS/'boundary_geometric_probe_v1/feasible_050.json')
    audit=read(RESULTS/'boundary_solver_check_v2/summary.json')
    def save(fig,name):
        fig.tight_layout()
        for extension in ['png','svg']: fig.savefig(output/f'{name}.{extension}',dpi=180)
        plt.close(fig)
    fig,ax=plt.subplots(figsize=(6,4.4))
    for row in screen['gridRows']:
        a,b,_,_=row['parameters']; ax.scatter(a*1000,b*1000,c='#2a7f62' if row['passed'] else '#b84a48',s=100,marker='s')
    ax.set(xlabel='Fixture A offset (mm)',ylabel='Fixture B offset (mm)',title='One fixed geometric path: 25 / 25 accepted at T = 7.5 s',xticks=[-20,-10,0,10,20],yticks=[-20,-10,0,10,20]); ax.grid(alpha=.15); save(fig,'common_path_grid')
    rows=sorted(screen['timingRows'],key=lambda r:r['T']); fig,axes=plt.subplots(1,2,figsize=(10,4))
    for key,label in [('maxSpeedUtilization','Leg speed'),('maxAccelerationUtilization','Leg acceleration'),('maxForceUtilization','Force')]:
        axes[0].plot([r['T'] for r in rows],[r['quality'][key] for r in rows],'o-',label=label)
    axes[0].axhline(1,color='black',linestyle='--',linewidth=1); axes[0].set(xlabel='Execution duration T (s)',ylabel='Peak constraint utilization'); axes[0].legend()
    for i,r in enumerate(rows): axes[1].barh(i,r['T'],color='#2a7f62' if r['passed'] else '#b84a48')
    axes[1].axvline(rows[0]['timeLowerBound'],color='#4a6b9a',linestyle='--',label=f"Variation bound: {rows[0]['timeLowerBound']:.3f} s")
    axes[1].set(yticks=range(len(rows)),yticklabels=[f"T={r['T']:g}: {'accepted' if r['passed'] else 'rejected'}" for r in rows],xlabel='Time (s)'); axes[1].legend(fontsize=8)
    save(fig,'duration_diagnostic')
    t=np.asarray(trajectory['t']); Q=np.asarray(trajectory['q']); fig,axes=plt.subplots(2,3,figsize=(12,6.5))
    axes[0,0].plot(t,Q[1]*1000,label='platform y'); axes[0,0].set(ylabel='Lateral position (mm)')
    axes[0,1].plot(t,np.rad2deg(Q[5])); axes[0,1].set(ylabel='Yaw (deg)')
    for j,gap in enumerate(trajectory['payloadGapLower']): axes[0,2].plot(t,np.asarray(gap)*1000,label=f'Fixture {j+1}')
    axes[0,2].axhline(10,color='black',linestyle='--',linewidth=1); axes[0,2].set(ylabel='Separation certificate (mm)'); axes[0,2].legend()
    for i in range(6):
        axes[1,0].plot(t,trajectory['legLength'][i],label=f'Leg {i+1}',linewidth=1)
        axes[1,1].plot(t,trajectory['legAcceleration'][i],linewidth=1)
        axes[1,2].plot(t,trajectory['force'][i],linewidth=1)
    axes[1,0].set(ylabel='Leg length (m)'); axes[1,0].legend(ncol=2,fontsize=7)
    axes[1,1].set(ylabel='Leg acceleration (m/s²)'); axes[1,2].set(ylabel='Required force (N)')
    for ax in axes.flat: ax.set_xlabel('Time (s)'); ax.grid(alpha=.15)
    fig.suptitle('Independent geometric candidate 050; full 5 ms physical check passed')
    save(fig,'representative_curves')
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig=plt.figure(figsize=(8,6)); ax=fig.add_subplot(111,projection='3d')
    for ob in trajectory['obstacles']:
        center=np.asarray(ob['center_S']); half=np.asarray(ob['halfSize']); R=np.asarray(ob['R_S'])
        vertices=np.array([center+R@(half*np.array(s)) for s in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]])
        faces=[[vertices[i] for i in ids] for ids in [[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[0,3,7,4],[1,2,6,5]]]
        ax.add_collection3d(Poly3DCollection(faces,facecolor='#c58455',edgecolor='#805332',alpha=.35))
    centers=np.asarray(trajectory['center']); axes_world=np.asarray(trajectory['axis']); ax.plot(*centers,color='black',linewidth=1)
    theta=np.linspace(0,2*np.pi,24); longitudinal=np.linspace(-.6,.6,2)
    for k in np.linspace(0,len(t)-1,5).astype(int):
        axis=axes_world[:,k]; side=np.cross(axis,[0,0,1]); side/=np.linalg.norm(side); up=np.cross(axis,side)
        mesh=centers[:,k,None,None]+axis[:,None,None]*longitudinal[None,:,None]+.15*(side[:,None,None]*np.cos(theta)[None,None,:]+up[:,None,None]*np.sin(theta)[None,None,:])
        ax.plot_surface(mesh[0],mesh[1],mesh[2],color='#467b9f',alpha=.2,linewidth=0)
    ax.set(xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',xlim=(-.9,.9),ylim=(-.75,.75),zlim=(.95,1.5),title='Full cylindrical payload at five poses; fixtures A / B')
    ax.set_box_aspect((1.8,1.5,.55))
    save(fig,'representative_geometry')
    fig,axes=plt.subplots(1,2,figsize=(9,4)); names=[r['variant'] for r in audit['records']]
    for i,r in enumerate(audit['records']):
        axes[0].bar(i,r['buildSeconds'],color='#7799b4',label='Build' if i==0 else None); axes[0].bar(i,r['solveSeconds'],bottom=r['buildSeconds'],color='#345c77',label='Solve' if i==0 else None)
        axes[1].scatter(i-.08,r['maxEquality'],marker='o',color='#b84a48',label='Equality' if i==0 else None); axes[1].scatter(i+.08,r['maxInequality'],marker='s',color='#8d683c',label='Inequality' if i==0 else None)
    axes[0].set(xticks=range(2),xticklabels=names,ylabel='Actual build + solve time (s)',title='Debug calls; not a formal speed comparison')
    axes[1].axhline(1e-6,color='black',linestyle='--'); axes[1].set(xticks=range(2),xticklabels=names,yscale='log',ylabel='Discrete equality / inequality residual',title='Both physical passes remain IHSID failures')
    for ax in axes: ax.legend(fontsize=8)
    save(fig,'solver_acceptance_audit')
    summary=dict(implementation='first_round_core',formalExperimentComplete=False,formalTrainingCalls=0,formalBenchmarkCalls=0,
                 geometricCandidates=probe['candidateCount'],geometricDenseAccepted=probe['qualified'],samePathGridAccepted=screen['sameFixedPathPassed'],
                 historyDiagnosticProcessed=8,historyDiagnosticExpected=1928,variationLowerBoundSeconds=rows[0]['timeLowerBound'],
                 smallestTestedFeasibleDuration=1.92,rejectedTestDuration=1.536,
                 stageGate='Do not expand training: reuse boundary and qualified IHSID labeling have not been established.',
                 deferredConditionalMethods=['M5_P','M5_R','M6_CVAE'],geometryScope='Declared idealized envelopes, not CAD verified')
    write(output/'implementation_status.json',summary)
    html='''<!doctype html><meta charset="utf-8"><title>双工装实验：实现与小试</title><style>body{font:16px/1.7 system-ui,"Microsoft YaHei";max-width:1000px;margin:36px auto;padding:0 20px;color:#24323e}img{max-width:100%;margin:14px 0}h1{font-size:26px}h2{font-size:20px}.note{background:#eef3f7;padding:14px 20px;border-left:4px solid #467b9f}</style>
<h1>双工装实验：首轮实现与有界小试</h1><p class="note">已实现首轮核心流程并完成接口与数值验证。正式新任务训练和全量实验均未启动；这些图不是扩散性能结果。</p>
<h2>当前最重要的发现</h2><p>独立几何扫描产生72条候选，选择的12条全部通过5 ms完整物理验收。同一条固定路径覆盖了25个工装偏移任务。因此，当前±20 mm、7.5 s范围尚未呈现路径复用边界。</p>
<img src="common_path_grid.png"><p>同一路径在1.92 s仍合格，1.536 s因腿运动限制失败。约0.481 s的总变差下界远低于这两个时长，不能据此证明任意重新计时不可能。</p><img src="duration_diagnostic.png">
<h2>真实轨迹与约束</h2><p>图中为普通圆柱载荷的几何候选。机构碰撞采用声明的理想化包络，尚未核实真实CAD；稠密数值检查不等于连续时间证明。</p><img src="representative_geometry.png"><img src="representative_curves.png">
<h2>IHSID三重门保留了失败</h2><p>匹配旧、新实现的两次调试求解，其重建轨迹通过物理检查，但求解状态、离散残差不合格，均按IHSID失败保留。图只说明判据，没有建立两实现的正式效率结论。</p><img src="solver_acceptance_audit.png">
<h2>实现与验证范围</h2><p>已接通双工装场景、共同验收、原始历史导入、四种适配、时间下界、非均匀重新计时、参数化IHSID、近邻/增强检索/局部回归/MLP/DDPM、共同精修与回退、分组数据生成、训练导出、验证选参、冻结协议、常驻/冷启动预算及统计入口。</p>
<p>12项MATLAB基础检查、10项集成检查、9项Python测试通过，另完成真实进程常驻与冷启动截止测试。MLP/DDPM训练测试使用合成接口数据，不是新实验训练结果。</p>
<p>旧训练库已按训练划分导入482条轨迹，仅对前2条的4种适配做了8项诊断，共1928项尚未全查；不能声称全库不可复用。</p>
<h2>尚未完成与下一步</h2><p>完整阶段B参考解矩阵、全库诊断、正式域标定、真实标签数据集、正式训练和全量比较尚未完成。重组与CVAE属于条件追加方法，当前未触发其实现阶段。应先标定更紧节拍/更宽工装范围，并得到严格合格的IHSID标签，不应直接启动600条件训练。</p>
<p>正式运行前还需完成：统一几何进度与腿运动的模式判据（目前为中心/轴向特征代理）；按不同有效模式数量触发追加标签；完整批内排序与候选级事件审计；全库均匀/非均匀重新计时诊断；离线12→16→24控制点增强以及完整E1—E5图表。当前是有测试覆盖的首轮实现，并非整个方案已验收。</p>
<h2>可复现入口</h2><p>在项目根目录运行 <code>python chapter3/boundary/run_boundary.py --help</code> 查看各阶段参数；<code>audit</code> 和 <code>test</code> 用于审计与测试。远程数值执行须使用已安装CasADi的MATLAB R2019a和环境变量 <code>STEWART_DEPENDENCY_ROOT</code> 指向 stewart-simscape-abfd。Python训练使用远程 stewart-abfd 环境。</p>
<p>本报告可用 <code>python chapter3/boundary/report_preflight.py</code> 从包内原始JSON重新生成。数值记录保留各次运行的源码哈希；旧调试记录不会因当前代码修正自动变成新版本结果。冻结正式协议前必须重新运行受影响检查，且不得跨版本混合统计。</p>
<p>轻量包保留新试运行原始轨迹、日志、源码与检查记录。约315 MB的旧库MAT副本仍在远程工作站，包内提供来源和校验清单。</p>'''
    (output/'implementation_report.html').write_text(html,encoding='utf-8')
    print(json.dumps(dict(output=str(output),figures=5,status=summary),ensure_ascii=True))


if __name__=='__main__': main()
