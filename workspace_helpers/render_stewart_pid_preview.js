const fs = require('fs');
const path = require('path');

const W = 1550;
const H = 850;
const sx = x => x * 100;
const sy = y => H - y * 100;
const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');

const parts = [];
const line = (x1, y1, x2, y2, arrow = false) => parts.push(
  `<line x1="${sx(x1)}" y1="${sy(y1)}" x2="${sx(x2)}" y2="${sy(y2)}" class="wire"${arrow ? ' marker-end="url(#arrow)"' : ''}/>`
);
const block = (x, y, w, h, lines, size = 27) => {
  const left = sx(x - w / 2), top = sy(y + h / 2);
  parts.push(`<rect x="${left}" y="${top}" width="${sx(w)}" height="${sx(h)}" class="block"/>`);
  const arr = Array.isArray(lines) ? lines : [lines];
  const gap = size * 1.25;
  const start = sy(y) - (arr.length - 1) * gap / 2;
  arr.forEach((t, i) => parts.push(`<text x="${sx(x)}" y="${start + i * gap}" class="cn blocktext" font-size="${size}">${esc(t)}</text>`));
};
const sum = (x, y) => {
  parts.push(`<circle cx="${sx(x)}" cy="${sy(y)}" r="32" class="block"/>`);
  line(x - .21, y - .21, x + .21, y + .21);
  line(x - .21, y + .21, x + .21, y - .21);
};
const dot = (x, y) => parts.push(`<circle cx="${sx(x)}" cy="${sy(y)}" r="5" fill="#000"/>`);
const text = (x, y, value, cls = 'cn', size = 25) => parts.push(`<text x="${sx(x)}" y="${sy(y)}" class="${cls}" font-size="${size}">${esc(value)}</text>`);

parts.push(`<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">
<defs><marker id="arrow" markerWidth="12" markerHeight="12" refX="10" refY="4" orient="auto" markerUnits="strokeWidth"><path d="M0,0 L10,4 L0,8 z" fill="#000"/></marker></defs>
<style>
  .wire{stroke:#000;stroke-width:3;fill:none;stroke-linecap:square;stroke-linejoin:miter}
  .block{fill:#fff;stroke:#000;stroke-width:3}
  text{fill:#000;text-anchor:middle;dominant-baseline:middle}
  .cn{font-family:"Microsoft YaHei","SimSun",sans-serif}
  .math{font-family:"Times New Roman",serif;font-style:italic}
  .en{font-family:"Times New Roman",serif}
  .blocktext{font-weight:400}
</style>
<rect width="100%" height="100%" fill="#fff"/>`);

const y = 4.8;
// Main flow
line(.45, y, 1.25, y, true);
block(2.15, y, 1.8, 1.05, ['运动学', '逆解'], 27);
line(3.05, y, 3.80, y, true);
sum(4.12, y);

// Error split and PID branches
line(4.44, y, 5.08, y);
dot(5.08, y);
line(5.08, y, 5.08, 6.30);
line(5.08, 6.30, 5.575, 6.30, true);
line(5.08, y, 5.575, y, true);
line(5.08, y, 5.08, 3.30);
line(5.08, 3.30, 5.575, 3.30, true);
block(6.35, 6.30, 1.55, .90, '比例  P', 25);
block(6.35, y, 1.55, .90, '积分  I', 25);
block(6.35, 3.30, 1.55, .90, '微分  D', 25);

// PID merge
line(7.125, 6.30, 8.62, 6.30);
line(8.62, 6.30, 8.62, 5.12, true);
line(7.125, y, 8.30, y, true);
line(7.125, 3.30, 8.62, 3.30);
line(8.62, 3.30, 8.62, 4.48, true);
sum(8.62, y);

line(8.94, y, 9.80, y, true);
block(10.80, y, 2.0, 1.05, ['Stewart', '平台'], 27);
line(11.80, y, 12.55, y);
dot(12.55, y);
line(12.55, y, 14.72, y, true);

// Feedback
line(12.55, y, 12.55, 1.70);
line(12.55, 1.70, 4.12, 1.70);
line(4.12, 1.70, 4.12, 4.48, true);

// Labels
text(.95, 5.42, 'r(t)', 'math', 40);
text(.75, 4.24, '期望轨迹', 'cn', 27);
text(3.63, 5.18, '+', 'en', 29);
text(3.72, 4.30, '−', 'en', 31);
text(4.65, 5.35, 'e(t)', 'math', 38);
text(6.35, 7.10, 'PID 控制器', 'cn', 28);
text(9.22, 5.38, 'u(t)', 'math', 38);
text(13.90, 5.42, 'y(t)', 'math', 40);
text(14.00, 4.25, '输出轨迹', 'cn', 27);
text(8.35, 1.98, '输出杆长', 'cn', 27);
text(7.75, .88, 'Stewart 平台 PID 闭环控制原理图', 'cn', 29);
text(7.75, .40, 'Closed-loop PID Control Principle of Stewart Platform', 'en', 27);

parts.push('</svg>');
const svg = parts.join('\n');
const svgPath = path.join(process.cwd(), 'Stewart平台_PID闭环控制原理图.svg');
const pngPath = path.join(process.cwd(), 'Stewart平台_PID闭环控制原理图.png');
fs.writeFileSync(svgPath, svg, 'utf8');
console.log(JSON.stringify({svgPath, pngPath, width: W, height: H}));
