from __future__ import annotations

import html
import math
import random
from pathlib import Path


ROOT = Path(r"D:\NPU\Matlab_code\stewart-simscape-diffusion\docs\figs")
ROOT.mkdir(parents=True, exist_ok=True)
STEM = "regression_vs_diffusion_wide"

BLUE = "#2679A9"
CYAN = "#50B4C9"
DEEP = "#174766"
PURPLE = "#7355B9"
MAGENTA = "#C365AE"
GREEN = "#219A6D"
RED = "#D47A7A"
MUTED = "#607687"
FAINT = "#DCE8EE"

parts: list[str] = []


def add(s: str) -> None:
    parts.append(s)


def esc(s: str) -> str:
    return html.escape(s, quote=True)


def line(x1, y1, x2, y2, color=BLUE, width=2, opacity=1, dash=None, marker=None):
    a = f' stroke-dasharray="{dash}"' if dash else ""
    b = f' marker-end="url(#{marker})"' if marker else ""
    add(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{width}" opacity="{opacity}" stroke-linecap="round"{a}{b}/>')


def path(d, color=BLUE, width=2, opacity=1, fill="none", dash=None, marker=None):
    a = f' stroke-dasharray="{dash}"' if dash else ""
    b = f' marker-end="url(#{marker})"' if marker else ""
    add(f'<path d="{d}" fill="{fill}" stroke="{color}" stroke-width="{width}" opacity="{opacity}" stroke-linecap="round" stroke-linejoin="round"{a}{b}/>')


def rect(x, y, w, h, fill="white", stroke="none", r=12, sw=1, opacity=1, dash=None):
    a = f' stroke-dasharray="{dash}"' if dash else ""
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" opacity="{opacity}"{a}/>')


def circle(x, y, r, fill="white", stroke="none", sw=1, opacity=1):
    add(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" opacity="{opacity}"/>')


def text(x, y, value, size=20, fill=DEEP, weight=400, anchor="start", italic=False, opacity=1, letter=None):
    s = f' font-style="italic"' if italic else ""
    l = f' letter-spacing="{letter}"' if letter is not None else ""
    add(f'<text x="{x}" y="{y}" fill="{fill}" font-size="{size}" font-weight="{weight}" text-anchor="{anchor}" opacity="{opacity}"{s}{l}>{esc(value)}</text>')


def tag(x, y, value, color=BLUE, fill="#EDF7FA", w=None):
    w = w or 13 * len(value) + 23
    rect(x, y, w, 29, fill, "none", 14)
    text(x + w / 2, y + 20, value, 18, color, 700, "middle", letter=0.8)


def chevron(x1, y1, x2, y2, color=BLUE, width=2.4, opacity=1):
    marker = "arrow-purple" if color == PURPLE else "arrow-green" if color == GREEN else "arrow-blue"
    line(x1, y1, x2, y2, color, width, opacity, marker=marker)


def tiny_vector(x, y, w, h, color, noisy=0, seed=1, show_nodes=False):
    rng = random.Random(seed)
    count = 9
    step = w / count
    for i in range(count):
        v = 0.24 + 0.67 * (0.5 + 0.5 * math.sin(i * 1.15 + 0.7))
        v = max(0.15, min(1, v + noisy * rng.uniform(-0.55, 0.55)))
        rect(x + i * step + 1, y + h * (1 - v), step - 3, h * v, color, "none", 3, opacity=0.43 + 0.46 * v)
        if show_nodes and i in (1, 3, 5, 7):
            circle(x + (i + 0.5) * step, y + h * (1 - v), 3.4, "white", color, 1.2)


def spline(x, y, w, h, color=BLUE, width=4, variant=0, faded=1, controls=False):
    offsets = [(0, 0, 0), (4, -8, 5), (-4, 9, -8), (6, 3, -4)]
    a, b, c = offsets[variant % 4]
    d = (f'M {x} {y+h*0.72+a} C {x+w*0.20} {y+h*0.18+b}, '
         f'{x+w*0.36} {y+h*0.95+c}, {x+w*0.52} {y+h*0.48} '
         f'S {x+w*0.82} {y+h*0.15-b}, {x+w} {y+h*0.35+a}')
    path(d, color, width, faded)
    if controls:
        pts = [(0.00,.72),(0.12,.47),(0.25,.43),(0.38,.77),(0.51,.48),(.65,.24),(.78,.22),(.89,.29),(1,.35)]
        for i, (px, py) in enumerate(pts):
            circle(x + w * px, y + h * py, 4.7 if 1 <= i <= 7 else 3.5,
                   "white", color, 1.7, 0.93)


def network(xs, cy, dims, helper_x=None, helper_offset=75, helper_label="线性层 + Tanh"):
    counts = [6, 7, 7, 7, 6]
    nodes: list[list[float]] = []
    for n in counts:
        ys = [cy - 48 + i * 19.2 for i in range(n)]
        nodes.append(ys)
    for layer in range(4):
        for i, ya in enumerate(nodes[layer]):
            for j, yb in enumerate(nodes[layer + 1]):
                if (i * 3 + j * 2 + layer) % 5 == 0:
                    continue
                line(xs[layer] + 6, ya, xs[layer + 1] - 6, yb, BLUE, 0.95, 0.10 if abs(i-j)>3 else 0.18)
    for k, (x, ys) in enumerate(zip(xs, nodes)):
        for i, y in enumerate(ys):
            circle(x, y, 5.7, "url(#nodeGrad)", BLUE if k in (0,4) else CYAN, 1.05)
        text(x, cy - 74, str(dims[k]), 18, DEEP, 700, "middle")
    text(helper_x if helper_x is not None else xs[2], cy + helper_offset, helper_label, 18, MUTED, 500, "middle")


def particle_cloud(cx, cy, color=PURPLE, n=32, seed=5, spread=37):
    rng = random.Random(seed)
    for i in range(n):
        a = rng.uniform(0, math.tau)
        r = spread * math.sqrt(rng.uniform(0.02, 1))
        circle(cx + math.cos(a) * r, cy + math.sin(a) * r,
               rng.uniform(1.6, 3.6), color, "none", opacity=rng.uniform(0.35, 0.85))


add('''<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="2400" height="800" viewBox="0 0 2400 800" role="img" aria-label="确定性回归与条件扩散轨迹生成架构图，最终共用 Stewart 物理验收">''')
add('''<defs>
<linearGradient id="nodeGrad" x1="0" y1="0" x2="1" y2="1"><stop stop-color="#FFFFFF"/><stop offset="0.55" stop-color="#E8F6FA"/><stop offset="1" stop-color="#BDDFED"/></linearGradient>
<linearGradient id="diffGrad" x1="0" y1="0" x2="1" y2="0"><stop stop-color="#7459BA"/><stop offset="1" stop-color="#C365AE"/></linearGradient>
<linearGradient id="branchBlue" x1="0" y1="0" x2="1" y2="0"><stop stop-color="#F9FCFD"/><stop offset="1" stop-color="#F1F9FC"/></linearGradient>
<linearGradient id="branchViolet" x1="0" y1="0" x2="1" y2="0"><stop stop-color="#FAF9FD"/><stop offset="1" stop-color="#F8F5FC"/></linearGradient>
<marker id="arrow-blue" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 1 1 L 9 5 L 1 9" fill="none" stroke="#2679A9" stroke-width="1.5"/></marker>
<marker id="arrow-purple" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 1 1 L 9 5 L 1 9" fill="none" stroke="#7355B9" stroke-width="1.5"/></marker>
<marker id="arrow-green" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 1 1 L 9 5 L 1 9" fill="none" stroke="#219A6D" stroke-width="1.5"/></marker>
</defs>''')
add('<style>text{font-family:Inter,"Microsoft YaHei","Noto Sans CJK SC",Helvetica,Arial,sans-serif}svg{background:#FFFFFF}</style>')
rect(0, 0, 2400, 800, "#FFFFFF", "none", 0)

# Section rhythm: one horizontal reading line, with two aligned model branches.
sections = [(74, 355, "01  教师与数据集"), (475, 690, "02  任务与轨迹表征"),
            (755, 1330, "03  学习模型"), (1390, 1975, "04  轨迹生成"),
            (2025, 2340, "05  物理验收")]
for x1, x2, label in sections:
    text(x1, 90, label, 20, MUTED, 700, letter=1.2)
    line(x1, 108, x2, 108, FAINT, 2)

# Teacher: orbiting search traces, not an algorithm expansion.
text(76, 186, "离线教师", 26, DEEP, 700)
circle(158, 317, 82, "#F7FBFD", "#C6E1EA", 1.8)
circle(158, 317, 58, "none", "#C9E4ED", 1.4)
path("M 107 343 C 125 315, 138 347, 153 302 S 183 267, 211 283", BLUE, 3.5)
path("M 111 287 C 135 296, 133 325, 164 327 S 186 354, 204 335", CYAN, 2.2, 0.75)
for cx, cy in [(109,343),(153,302),(211,283),(164,327),(204,335)]:
    circle(cx, cy, 5.2, "white", BLUE, 1.7)
text(158, 438, "IHSID", 32, DEEP, 800, "middle")
text(158, 471, "约束轨迹优化", 18, MUTED, 500, "middle")

# Dataset, visually represented as trajectory cards.
chevron(244, 317, 275, 317)
text(284, 210, "成功轨迹", 23, DEEP, 700)
text(284, 247, "数据集", 23, DEEP, 700)
for dx, dy, op in [(10,-12,0.62),(5,-6,0.81),(0,0,1)]:
    rect(286+dx, 286+dy, 148, 113, "white", "#C5DFE9", 10, 1.5, op)
    spline(301+dx, 308+dy, 117, 60, BLUE, 2.2, variant=(dx//5), faded=op)
text(286, 437, "任务–轨迹配对", 18, MUTED)

# Dataset forks to representations.
path("M 445 339 H 464 V 271 H 490", BLUE, 2.4, marker="arrow-blue")
path("M 445 349 H 464 V 553 H 490", BLUE, 2.4, marker="arrow-blue")

# Task feature vector.
text(498, 200, "任务条件  c", 25, DEEP, 750)
text(498, 230, "99 维", 21, BLUE, 750)
rect(498, 253, 198, 78, "#F1FAFC", "#D0E8EF", 11, 1)
tiny_vector(511, 267, 171, 50, BLUE, noisy=0.38, seed=8)
text(498, 357, "起点 / 目标 / 时间", 18, MUTED)
text(498, 382, "障碍物 / 载荷", 18, MUTED)

# Optimal B-spline trajectory representation.
text(498, 474, "最优轨迹  y₀", 24, DEEP, 750)
text(498, 504, "60 维 B-spline 参数", 20, BLUE, 650)
rect(497, 523, 207, 104, "#F8FCFD", "#D8E9EF", 11, 1)
spline(509, 542, 181, 70, BLUE, 3.2, controls=True)

# Two quiet branch surfaces and aligned networks.
rect(744, 154, 601, 252, "url(#branchBlue)", "#D9EBF0", 18, 1.3)
rect(744, 417, 601, 338, "url(#branchViolet)", "#E5DFF0", 18, 1.3)
text(763, 191, "确定性回归 Regression", 29, DEEP, 760)
tag(1216, 170, "训练", BLUE, "#E7F4F8", 107)
text(763, 454, "条件扩散 Diffusion", 29, DEEP, 760)
tag(1216, 433, "训练", PURPLE, "#F0EBF8", 107)

# Regression preprocessing and visible multilayer perceptron.
rect(761, 244, 150, 62, "#EAF6F9", "#CAE5EC", 14, 1.1)
for i, hh in enumerate((18,29,39,25)):
    rect(775+i*20, 290-hh, 11, hh, BLUE, "none", 3, opacity=0.36+i*0.13)
text(836, 335, "标准化", 18, BLUE, 650, "middle")
chevron(912, 275, 970, 275)
network([990,1065,1140,1215,1290], 292, [99,256,256,256,60])
text(1295, 352, "ŷ₀", 20, BLUE, 700, italic=True)
text(763, 386, "输入：任务条件 c（99 维）", 18, BLUE, 600)

# Diffusion's forward process occupies a continuous, compact strip before the aligned MLP.
text(767, 493, "噪声等级", 18, PURPLE, 700)
path("M 773 523 C 782 501, 790 541, 799 518 S 816 502, 824 519", PURPLE, 2.2)
text(835, 524, "t", 18, PURPLE, 600, italic=True)
path("M 851 518 C 899 506, 947 510, 976 536", PURPLE, 2.0, 0.82, marker="arrow-purple")
text(764, 572, "前向加噪", 20, PURPLE, 750)
for i, (xx, noise, op) in enumerate([(765,0.05,0.55),(825,0.42,0.75),(885,1.0,1)]):
    rect(xx, 588, 42, 47, "#F7F1FB", "#D8C9E9", 7, 1, op)
    tiny_vector(xx+4, 595, 34, 32, PURPLE, noisy=noise, seed=i+20)
chevron(810, 612, 821, 612, PURPLE, 1.5)
chevron(870, 612, 881, 612, PURPLE, 1.5)
text(764, 659, "纯净", 18, MUTED)
text(825, 659, "过渡", 18, MUTED)
text(885, 659, "噪声", 18, MUTED)
particle_cloud(798, 715, PURPLE, 24, 36, 21)
text(833, 711, "高斯噪声", 18, PURPLE, 650)
path("M 812 685 V 639", PURPLE, 1.7, 0.8, marker="arrow-purple")
path("M 928 612 C 950 612, 949 582, 976 575", PURPLE, 2.0, 0.82, marker="arrow-purple")
network([990,1065,1140,1215,1290], 565, [161,256,256,256,60], helper_x=1140, helper_offset=94, helper_label="矩阵")
text(1140, 695, "预测噪声  ε̂", 18, PURPLE, 700, "middle")
text(933, 738, "输入：c（99）· yₜ（60）· 噪声等级编码（2）", 18, MUTED, 550)

# Inference zone: direct one-shot regression, iterative diffusion, then distinct candidates.
tag(1393, 157, "推理", BLUE, "#E7F4F8", 105)
path("M 1345 288 H 1392", BLUE, 2.5, marker="arrow-blue")
text(1394, 229, "预测轨迹  ŷ₀", 22, DEEP, 700)
rect(1394, 245, 443, 116, "#F7FBFD", "#D6EAF0", 14, 1.2)
spline(1434, 263, 362, 76, BLUE, 4.4, controls=True)
text(1395, 393, "同一条件  →  单条轨迹", 21, BLUE, 700)
chevron(1837, 303, 2020, 303, BLUE, 2.4)

tag(1393, 427, "推理", PURPLE, "#F1EBF8", 105)
path("M 1345 565 H 1387", PURPLE, 2.5, marker="arrow-purple")
text(1393, 487, "迭代反向去噪", 22, PURPLE, 750)
text(1394, 515, "从噪声逐步形成轨迹", 18, MUTED, 550)
states = [(1413,1.0,"噪声"),(1504,.68,"粗结构"),(1595,.42,"部分轨迹"),(1686,.19,"精细轨迹"),(1777,.02,"生成")]
for i, (x, noisiness, label) in enumerate(states):
    rect(x, 540, 72, 74, "#F7F2FB", "#E0D2EC", 11, 1.1)
    if i == 0:
        particle_cloud(x+36, 577, PURPLE, 25, 80, 27)
    else:
        spline(x+9, 554, 54, 45, PURPLE if i<4 else MAGENTA, 2.6, variant=1,
               faded=0.38+0.15*i)
        particle_cloud(x+36, 577, PURPLE, round(16*noisiness), 86+i, 24)
    text(x+36, 642, label, 18, MUTED if i < 4 else PURPLE, 600, "middle")
    if i < 4:
        chevron(x+74, 576, states[i+1][0]-5, 576, PURPLE, 1.9)

# Multiple samples of the same condition. The paths are visibly different.
path("M 1852 578 H 1871", PURPLE, 2.3, marker="arrow-purple")
text(1879, 480, "多条", 20, PURPLE, 750)
text(1879, 514, "候选轨迹", 20, PURPLE, 750)
rect(1872, 529, 127, 126, "#FBF8FD", "#E7D9EF", 12, 1)
for i, (yy, col, op) in enumerate([(541,PURPLE,.85),(568,MAGENTA,.70),(595,"#5E6EC0",.7)]):
    spline(1882, yy, 106, 38, col, 2.7, variant=i+1, faded=op)
text(1873, 692, "相同条件 c", 18, MUTED)
text(1873, 717, "不同噪声采样", 18, MUTED)
chevron(1999, 590, 2020, 590, PURPLE, 2.4)

# Shared physics: a compact constraint review, without an inaccurate platform sketch.
rect(2025, 156, 314, 591, "#FAFCFD", "#D8E5E9", 18, 1.3)
text(2045, 197, "Stewart 统一", 25, DEEP, 750)
text(2045, 229, "物理验收", 25, DEEP, 750)
text(2045, 258, "两种方法共用", 18, MUTED, 500)
for y, label in [(361, "支链运动约束"), (427, "力与奇异性约束"), (493, "碰撞间隙")]:
    circle(2056, y-6, 5.5, "#E9F5F8", BLUE, 1.4)
    text(2076, y, label, 21, DEEP, 600)
    line(2076, y+17, 2304, y+17, FAINT, 1.2)
text(2045, 553, "候选筛选", 18, MUTED, 550)
text(2071, 591, "✓", 27, GREEN, 800)
text(2122, 591, "×", 27, RED, 800)
text(2173, 591, "✓", 27, GREEN, 800)
text(2044, 644, "可行轨迹", 22, GREEN, 760)
rect(2043, 660, 272, 56, "#F1FAF6", "#CBE8D9", 10, 1)
spline(2056, 667, 245, 42, GREEN, 3.6, controls=False)

add('</svg>')
svg = "\n".join(parts) + "\n"
(ROOT / f"{STEM}.svg").write_text(svg, encoding="utf-8")
page = f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Regression 与 Diffusion 轨迹生成架构图</title><style>html,body{{margin:0;width:100%;height:100%;background:#fff}}body{{display:grid;place-items:center}}svg{{display:block;width:100vw;height:100vh;object-fit:contain}}@media print{{@page{{size:landscape;margin:0}}}}</style></head><body>{svg}</body></html>'''
(ROOT / f"{STEM}.html").write_text(page, encoding="utf-8")
print(ROOT)
