from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
import shutil, json, subprocess, hashlib, tempfile, re
import pymupdf as fitz

BASE=Path(__file__).resolve().parents[1]
WORK=BASE/'_work'
report=json.loads((WORK/'verification_report.json').read_text('utf8'))
assert report['pass']
shutil.copyfile(WORK/'main_taes.pdf',BASE/'main_taes.pdf')
(BASE/'audit').mkdir(exist_ok=True)
shutil.copyfile(WORK/'source_manifest.json',BASE/'audit/source_manifest.json')

readme='''TAES 论文源码编译说明

文件
  main_taes.tex      含全文、公式、原始参考文献文字的主文件
  IEEEtaes.cls       用户提供的类文件，字节未改动
  figures/          从 main.docx 原样提取的 8 个 PNG 文件
  compile.ps1       Windows PowerShell 两遍编译脚本
  核对说明.txt        已执行检查、允许的样式变化及原稿问题说明
  audit/            逐段、逐式、图片哈希及复编译核对记录

编译要求
  XeLaTeX；建议完整 MiKTeX 或 TeX Live。
  宏包：color、array、amsthm、graphicx、amsmath、amssymb、newunicodechar、
        placeins、textcomp，以及类文件自身加载的 wrapfig、hyperref、url 等。
  正文、标题和数学字体沿用原类文件的 TeX 字体。
  系统需有 Times New Roman 和 SimSun（宋体），仅用于原文中的少量特殊
  Unicode 字形：Λ、∆、Unicode 连字符、全角逗号；未替换为近似字符。
  上述系统字体随本机 Windows 提供，未作为字体文件重新分发。

在本目录运行两遍
  xelatex -interaction=nonstopmode -halt-on-error -file-line-error -no-shell-escape main_taes.tex
  xelatex -interaction=nonstopmode -halt-on-error -file-line-error -no-shell-escape main_taes.tex

也可在已允许执行本地脚本的 PowerShell 中运行 .\\compile.ps1。
不需要运行 BibTeX 或 Biber；参考文献文字直接存放于主文件中，避免格式化工具改写原文。
无需访问微信临时目录或原始 Word 文件；所有论文资源路径均相对本目录。

保真约定
  Word 的 LaTeX 公式文本转换为排版公式；MathType 的 user1 样式标记仅显示
  其参数内容，因此原稿公式 (20) 中的两个等号均保留。
  原文缺失的正文公式引用不补写；作者、机构、收稿日期的占位内容照录。
  章节、图表标签、标题大小写显示、Index Terms 标签按 TAES 模板排版。
  Table 1 / Table 2 的明确正文引用同步显示为 Table I / Table II。
  30 个公式编号沿用 Word 原有编号域的 1–30，不改动公式内容。
  末页用类文件提供的 IEEEtriggeratref 平衡两栏。以后增删内容后，可调整
  主文件中的 IEEEtriggeratref{32}；这只改变栏间分页。

类文件兼容处理
  保留原类文件。主文件在 XeLaTeX 下选用其原 OT1 字体，显式应用类文件最终
  的 Letter 纸张尺寸，并在最后跨栏图表页后恢复类文件自身的正常栏高。
  原类文件内部声明的名称为 IEEEphot，因而会出现类名提示；这来自原文件，
  并不代表实际排版采用另一个期刊模板。正文样式参数仍取自所提供的 IEEEtaes.cls。
'''
(BASE/'README_compile.txt').write_text(readme,encoding='utf-8-sig')
ps='''$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    for ($pass = 1; $pass -le 2; $pass++) {
        & xelatex -interaction=nonstopmode -halt-on-error -file-line-error -no-shell-escape main_taes.tex
        if ($LASTEXITCODE -ne 0) { throw "XeLaTeX failed on pass $pass" }
    }
    Write-Output 'Created main_taes.pdf'
} finally {
    Pop-Location
}
'''
(BASE/'compile.ps1').write_text(ps,encoding='utf-8-sig')

notes='''TAES 转换与核对说明
日期：2026-11-11

交付结果
  main_taes.tex、main_taes.pdf（13 页）、TAES_source_package.zip。
  源码包内有原始 IEEEtaes.cls、全部 8 幅原图和编译说明。

内容核对
  源文件 main.docx、TAES_template.tex、IEEEtaes.cls 的 SHA-256 均未改变。
  255 个非空正文层级块全部映射；以 164 组可独立核对单元检查文字。
  135 处行内/独立公式全部核对数学记号序列，矩阵行列分隔也单独核对。
  2 张数据表保留全部单元格及原合并关系；第 3 个 Word 表格作为图 6、图 7
  的布局容器处理，没有漏掉一张数据表。
  39 条参考文献保留原有文字、标点、顺序和 DOI 拼写，没有运行文献重写工具。
  8 幅 PNG 原样提取，文件 SHA-256 与 Word 内嵌文件一致；PDF 内解码后的
  图片像素也逐一与原图相同。排版只改变等比例显示尺寸和页面位置。
  文字检查排除数学环境、已授权的模板标签和排版空白，未发现其他文字差异。
  公式检查只允许排版断行、对齐空白、矩阵列间距和外层数学环境变化。
  附加检查通过不等于重新证明论文结论；本文未进行学术修改或事实纠错。

模板核对
  实际使用用户提供的类文件，未修改其任何字节。
  与提供的 TeX 模板的编译基准比较：纸张均为 Letter（612 × 792 PDF 点），
  正文、标题、摘要、题注、参考文献使用相同的字体家族与字号体系。
  章节及图表编号、标题的大小写显示、关键词标签和旧期刊标识，按已确认的
  TAES 迁移边界处理；主文件保留原始标题文字，大小写仅由排版命令控制。
  Word 没有提供的卷期、出版年份和论文 DOI 留空，没有沿用模板的示例数据。
  原稿致谢仍在正文末尾，内容没有因模板中的写作建议而移动或改写。
  所有页面已渲染检查；无内容越栏、缺字、缺图、未定义命令或未解析引用。
  编译日志仍有原类文件的名称提示及部分 Underfull 排版提示，并非零警告。

按要求保留的原稿情况
  1. 正文中 7 处 Eq. 引用为空，包括 “The Eq.,”、“Eq. .” 和 “Eq..”。
     源块：46、108、110、114、157、164、184。没有依据编号域补写这些空缺。
  2. 公式 (20) 的原始片段为 \\hat X\\user1{ = } = ...；两个等号均照录。
  3. 在 H_{ij,k} 那一行，原稿两个中间项都写为 H_{ij}^{(i)}；没有擅改成 j。
  4. 原文 “whererepresent”、“Jaccobian”、“Eular angel”等文字照录。
  5. 参考文献 [9] 中 “https://do i.org/...” 的空格，以及其他原有的
     缩写、空格、大小写、标点和重复条目，均未修正。
  6. 作者 XXX、机构 XXX、日期 xx 等占位内容保留。

审计记录
  audit/source_manifest.json：源段落、公式、表格、图片及输入哈希清单。
  audit/verification_report.json：逐项文字/公式/图片检查结果。
  audit/recompile_report.json：从源码压缩包解压后的独立目录复编译结果。
'''
(BASE/'核对说明.txt').write_text(notes,encoding='utf-8-sig')
report['visual_review']={'pages_checked':list(range(1,14)),'method':'rendered page images inspected; final changed pages re-inspected','clipping_or_overlap_found':False}
(BASE/'audit/verification_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')

def source_files():
    return [BASE/'main_taes.tex',BASE/'IEEEtaes.cls',BASE/'README_compile.txt',BASE/'compile.ps1',BASE/'核对说明.txt',*sorted((BASE/'figures').glob('*.png')),*sorted((BASE/'audit').glob('*.json'))]
zip_path=BASE/'TAES_source_package.zip'
def make_zip():
    with ZipFile(zip_path,'w',ZIP_DEFLATED) as zz:
        for f in source_files():zz.write(f,f.relative_to(BASE).as_posix())
make_zip()
isolated=Path(tempfile.mkdtemp(prefix='standalone_',dir=WORK))
with ZipFile(zip_path) as zz:
    assert zz.testzip() is None
    for n in zz.namelist():assert (isolated/n).resolve().is_relative_to(isolated.resolve())
    zz.extractall(isolated)
logs=[]
for pass_id in (1,2):
    r=subprocess.run(['xelatex','-recorder','-interaction=nonstopmode','-halt-on-error','-file-line-error','-no-shell-escape','main_taes.tex'],cwd=isolated,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=120)
    (isolated/f'compile-pass-{pass_id}.txt').write_bytes(r.stdout)
    assert r.returncode==0,r.stdout.decode(errors='replace')[-2000:]
orig=fitz.open(BASE/'main_taes.pdf');other=fitz.open(isolated/'main_taes.pdf')
assert len(orig)==len(other)==13
page_checks=[]
for i,(a,b) in enumerate(zip(orig,other),1):
    x=a.get_pixmap(matrix=fitz.Matrix(1.5,1.5));y=b.get_pixmap(matrix=fitz.Matrix(1.5,1.5))
    item={'page':i,'size_equal':a.rect==b.rect,'text_equal':a.get_text()==b.get_text(),'pixels_equal':x.samples==y.samples}
    assert all(item[k] for k in ['size_equal','text_equal','pixels_equal']),item
    page_checks.append(item)
fls=(isolated/'main_taes.fls').read_text(errors='replace')
assert 'Wechat' not in fls and 'RWTemp' not in fls
recompile={'pass':True,'source':'TAES_source_package.zip extracted into a fresh directory','passes':2,'compiler':'XeLaTeX / MiKTeX 26.5','source_directory_dependency':False,'system_fonts_required':['Times New Roman','SimSun'],'pages':page_checks}
(BASE/'audit/recompile_report.json').write_text(json.dumps(recompile,ensure_ascii=False,indent=2),encoding='utf8')
make_zip()
with ZipFile(zip_path) as zz:assert zz.testzip() is None
hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in [BASE/'main_taes.tex',BASE/'main_taes.pdf',zip_path]}
(BASE/'audit/deliverable_hashes.json').write_text(json.dumps(hashes,indent=2),encoding='utf8')
print(json.dumps({'pages':13,'source_files':len(source_files()),'independent_recompile':True,'all_pages_pixel_identical':True,'deliverables':[{ 'name':f.name,'bytes':f.stat().st_size} for f in [BASE/'main_taes.tex',BASE/'main_taes.pdf',zip_path,BASE/'核对说明.txt']]},ensure_ascii=False))
