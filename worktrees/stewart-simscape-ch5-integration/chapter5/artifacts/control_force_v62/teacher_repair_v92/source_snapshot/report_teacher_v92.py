"""Keep V90/V91 failures visible and report V92 without mixing timing versions."""
from pathlib import Path
import compare_teacher_v90 as compare
import plot_teacher_v90 as plot
import audit_teacher_safety_v91 as safety


def main():
    compare.main('v92')
    out=compare.BASE/'teacher_repair_v92';safety.OUT=out;safety.main();plot.main('v92')
    file=out/'index.html';text=file.read_text(encoding='utf-8')
    old='<p>物理限值、检查容差、四组初值、QP 和 LQI 未修改。</p>'
    new='''<p>最终 V92 使用原 V89 安全 QP、LQI、四组 IHSID 初值、物理限值和检查容差。只修改调度：复用已验证退避轨迹时，保留原有到达预算续期事件，避免把原可于 11.4 s 启动的目标规划推迟到 12.6 s。</p>
    <p>失败版本完整保留：V90 横穿在 14.16 s 加速度超限；V91 修复投影后硬约束全部通过、回退为零，但终端保持不足 0.5 s，仍未验收。V92 不使用 V91 的约束处理修改。</p>
    <p><b>验证来源：</b>横穿重新执行完整 V92 Simscape；两组环绕沿用 V90 完整时序，实际 V92 函数逐一核对所有被复用的状态，确认新增预算分支不会改变原调度路径，且安全过滤器仍是 V89。两组环绕 V89/V90 的全部 751 拍状态、指令及观测逐项差异均为零。等价回放不是新增完整仿真，计时仍归属 V90，不合并为同一版本的正式实验结果。</p>
    <p><a href="budget_audit.json">到达预算与调度等价核对</a> · <a href="safety_margins.json">完整物理裕度复算</a> · <a href="safety_margins.csv">物理裕度 CSV</a> · <a href="../teacher_repair_v90/trajectory_equivalence_v89_v90.json">原始状态和力指令逐项比较</a> · <a href="../teacher_repair_v90/timing_comparability.json">历史计时波动与迭代数</a></p>'''
    assert old in text;text=text.replace(old,new)
    text=text.replace('V90 仅改变基线重规划调度','V92 改变基线重规划调度并保留到达预算事件')
    file.write_text(text,encoding='utf-8')


if __name__=='__main__':main()
