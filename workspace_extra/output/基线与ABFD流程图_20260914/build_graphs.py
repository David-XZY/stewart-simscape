import json
from pathlib import Path

OUT = Path(__file__).resolve().parent

def node(id, text, x, y, color='blue', w=3.05, h=1.05, master='Process'):
    return dict(id=id,text=text,x=x,y=y,w=w,h=h,color=color,master=master)
def edge(a,b,label='',side=None,dashed=False):
    return dict(a=a,b=b,label=label,side=side,dashed=dashed)
def label(text,x,y,w=14,size=12,color='muted',h=.45,bold=False):
    return dict(text=text,x=x,y=y,w=w,h=h,size=size,color=color,bold=bold)

def make(kind):
    baseline=kind=='baseline'
    g=dict(name='基线主线流程图' if baseline else 'ABFD实验流程图',width=16,height=12,nodes=[],edges=[],labels=[])
    n,e,l=g['nodes'],g['edges'],g['labels']
    l.extend([
        label('Stewart 平台｜基线主线流程' if baseline else 'Stewart 平台｜ABFD 实验流程',8,11.45,size=25,color='ink',bold=True),
        label('stewart-simscape · 顺序规划与控制 · 图中展开计算力矩–LQI 力输入主链' if baseline else 'stewart-simscape-abfd · 图中展开 EABFD 全模块组合；各对照组按配置启用',8,10.95,size=12),
        label('01  离线规划与控制准备',8,10.5,size=13,color='blue',bold=True),
        label('02  在线闭环执行',8,6.4,size=13,color='blue',bold=True),
        label('03  结果评价与记录',8,2.45,size=13,color='blue',bold=True),
    ])
    if baseline:
        n.extend([
            node('task','任务与边界条件\n起终位姿、障碍物、插入要求',2,9.65),
            node('model','名义几何与动力学模型\n运动学、力映射、工程约束',6,9.65),
            node('plan','IHSID 轨迹优化\n状态 / 加速度 / 腿力 / 分离证书',10,9.65),
            node('accept','稠密后验验收\n动力学、碰撞及执行器约束\n失败：保存诊断并终止',14,9.65,'green',h=1.25),
            node('bundle','参考轨迹导出与重建\n位姿、速度、加速度、腿力',14,7.85,'green'),
            node('design','工作点线性化与 LQI 设计\n观测器 / 积分器 / 力映射\n检查局部闭环稳定性',10,7.85,h=1.25),
            node('reference','按仿真时刻读取参考\nqᵣ、q̇ᵣ、q̈ᵣ',14,5.45),
            node('controller','计算力矩前馈 + LQI 反馈\nFnom = FCT + FLQI\n严格 CLF-CBF-QP：按入口启用',10,5.45,h=1.3),
            node('actuator','六支链力执行器\n按工况设置限幅、延迟与动态',6,5.45),
            node('plant','Simscape 多体平台\n平台、支链、载荷与重力',2,5.45),
            node('measure','传感与日志采样\n平台位姿、支链运动与实际力',2,3.6,'green'),
            node('observer','LQI 内部输出反馈补偿器\n线性 Luenberger 观测器\n泄漏积分与反馈限幅回算',6,3.6,h=1.3),
            node('metrics','闭环指标与硬约束评价\n跟踪误差、腿力、速度、加速度',2,1.55,'green'),
            node('result','保存仿真日志、指标与图表\n分别记录通过、失败及诊断',7.4,1.55,'green',w=5.6),
        ])
        l.extend([
            label('先生成并验收参考，再设计与运行控制器。\n其他独立入口：腿长 PID、长度串级、位姿外环。',4.5,8.05,w=7,size=13,h=.9),
            label('位姿误差由参考与测量构成；\n观测器状态属于线性补偿器坐标。',11.9,3.6,w=6.3,size=12,h=.85),
            label('范围：工程结构与信号关系；图示不代表所有控制入口同时运行。',8,.45,size=11),
        ])
        e.extend([edge('task','model'),edge('model','plan'),edge('plan','accept'),edge('accept','bundle','通过'),edge('bundle','design'),edge('design','controller','设计参数',dashed=True),edge('bundle','reference'),edge('reference','controller'),edge('controller','actuator','命令腿力'),edge('actuator','plant','实际腿力'),edge('plant','measure'),edge('measure','observer','位姿测量'),edge('observer','controller','反馈修正',[1,.5,.5,0]),edge('measure','metrics'),edge('metrics','result')])
    else:
        n.extend([
            node('task','统一任务与实验配置\n工况、状态来源、预算与门槛',2,9.65),
            node('model','B｜辨识与残差动力学\n物理参数辨识 + 静态残差\n形成统一动力学接口',6,9.65,'purple',h=1.3),
            node('plan','F｜学习生成轨迹初值\n多候选 → 可跟踪性排序\n选优后交给 IHSID 精修',10,9.65,'purple',h=1.3),
            node('accept','A｜轨迹与增益交替优化\n冻结增益求轨迹 ↔ 更新增益\n信赖域验收，保留上一可行解',14,9.65,'purple',h=1.3),
            node('bundle','C² 连续参考与稠密验收\nHS 节点 + 中点 → 五次 Hermite\n失败：保存诊断并终止',14,7.85,'green',h=1.3),
            node('design','D｜几何控制增益准备\nSO(3) 误差与空间加速度反馈\n参考沿线预计算 TVLQR 增益',10,7.85,'purple',h=1.3),
            node('reference','读取一致的执行参考\n参考与控制采用同一模型版本',14,5.45),
            node('controller','D｜几何逆动力学控制\n参考 + 当前状态 + 调度增益\n输出六维命令腿力',10,5.45,'purple',h=1.3),
            node('actuator','六支链非理想力执行器\n保留真实物理限幅与动态',6,5.45),
            node('plant','Simscape 多体实验对象\n共同工况下施加物理失配',2,5.45),
            node('measure','测量与区间力积分\n当前位姿、已完成区间 ∫Factual dt',2,3.6,'green',h=1.15),
            node('observer','状态来源二选一\n直接全状态 / 单一外部 EKF\nEKF：区间预测 → 位姿校正',6,3.6,h=1.3),
            node('metrics','统一验收与组间比较\n工程约束、反馈预算、在线 P95\n适用时检查联合误差管覆盖率',2,1.55,'green',h=1.3),
            node('result','保存原始日志、模型与配置指纹、分项结果\n区分执行完成、验收失败、异常和未完成',7.4,1.55,'green',w=5.6),
        ])
        l.extend([
            label('E0 对照：解析模型 + IHSID + 离散 LQI；\n研究基线使用统一全状态接口，不等同于原主线。\n实验矩阵：8 组主实验 + 5 组必要对照。',4.55,7.95,w=7.2,size=12,h=1.25),
            label('EKF 使用上一完整区间的实际力积分；\n直接状态通道不经过 EKF。\n采样周期默认 10 ms。',12.95,3.6,w=5.3,size=11,h=1.05),
            label('QP、DOB、抗饱和与 Governor 默认关闭；各模块启用关系由实验矩阵决定。图示不表示性能已获验收。',8,.45,size=11),
        ])
        e.extend([edge('task','model'),edge('model','plan','所选模型'),edge('plan','accept','候选初值'),edge('accept','bundle'),edge('bundle','design','已验收参考'),edge('design','controller','增益调度',dashed=True),edge('bundle','reference','通过'),edge('reference','controller'),edge('controller','actuator','命令腿力'),edge('actuator','plant','实际腿力'),edge('plant','measure'),edge('measure','observer','观测输入'),edge('observer','controller','x 或 x̂',[1,.5,.5,0]),edge('plant','observer','直接状态',[1,.2,0,.8],True),edge('measure','metrics'),edge('metrics','result')])
    (OUT/f'{kind}.graph.json').write_text(json.dumps(g,ensure_ascii=False,indent=2),encoding='utf-8')
for kind in ['baseline','experiment']:make(kind)
