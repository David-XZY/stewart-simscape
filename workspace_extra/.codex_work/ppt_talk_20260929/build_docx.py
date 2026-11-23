from pathlib import Path
import json, re, hashlib
from collections import Counter
from docx import Document
from docx.shared import Inches, Pt, RGBColor, Cm
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.opc.constants import RELATIONSHIP_TYPE as RT

WORK = Path(__file__).parent
OUT = Path(r'D:\NPU\组会PPT\26.9.29组会_第一人称通俗讲稿.docx')
text = (WORK/'manuscript.txt').read_text(encoding='utf-8')
doc = Document()
sec = doc.sections[0]
sec.page_width = Inches(8.5)
sec.page_height = Inches(11)
sec.top_margin = Cm(2.1)
sec.bottom_margin = Cm(2.0)
sec.left_margin = Cm(2.25)
sec.right_margin = Cm(2.25)
sec.header_distance = Cm(.85)
sec.footer_distance = Cm(.85)

def font(style, size=12, east='宋体', bold=False):
    style.font.name='Calibri'
    style.font.size=Pt(size)
    style.font.bold=bold
    style.font.color.rgb=RGBColor(0,0,0)
    rp=style.element.get_or_add_rPr()
    rf=rp.find(qn('w:rFonts'))
    if rf is None:
        rf=OxmlElement('w:rFonts'); rp.insert(0,rf)
    for attr,val in [('ascii','Calibri'),('hAnsi','Calibri'),('eastAsia',east)]:
        rf.set(qn('w:'+attr),val)
    for name in ['asciiTheme','hAnsiTheme','eastAsiaTheme','cstheme']:
        rf.attrib.pop(qn('w:'+name),None)

font(doc.styles['Normal'],12)
n=doc.styles['Normal'].paragraph_format
n.line_spacing=1.35
n.space_after=Pt(5)
n.first_line_indent=Pt(24)
n.widow_control=True
for name,size in [('Title',23),('Subtitle',12),('Heading 1',17),('Heading 2',13)]:
    font(doc.styles[name],size,'微软雅黑',name!='Subtitle')
    pf=doc.styles[name].paragraph_format
    pf.first_line_indent=Pt(0)
    pf.space_before=Pt(14 if name=='Heading 2' else 12)
    pf.space_after=Pt(7)
    pf.keep_with_next=True
    pf.line_spacing=1.2
for name in ['Caption','Header','Footer']:
    font(doc.styles[name],9,'宋体')
    doc.styles[name].paragraph_format.first_line_indent=Pt(0)
    doc.styles[name].paragraph_format.space_after=Pt(0)
    doc.styles[name].paragraph_format.line_spacing=1.15

# Explicitly remove borders and theme colors from heading styles.
for name in ['Title','Subtitle','Heading 1','Heading 2']:
    el=doc.styles[name].element
    for b in el.xpath('./w:pPr/w:pBdr'):
        b.getparent().remove(b)
    for c in el.xpath('./w:rPr/w:color'):
        for k in list(c.attrib):
            if k!=qn('w:val'): del c.attrib[k]
        c.set(qn('w:val'),'000000')

head=sec.header.paragraphs[0]
head.text='基于扩散模型的轨迹规划  ·  组会口述讲稿'
head.alignment=WD_ALIGN_PARAGRAPH.RIGHT
foot=sec.footer.paragraphs[0]
foot.alignment=WD_ALIGN_PARAGRAPH.CENTER
foot.add_run('第 ')
fld=OxmlElement('w:fldSimple');fld.set(qn('w:instr'),'PAGE');foot._p.append(fld)
foot.add_run(' 页')

def small_par(s, keep=False):
    p=doc.add_paragraph()
    p.paragraph_format.first_line_indent=Pt(0)
    p.paragraph_format.line_spacing=1.2
    p.paragraph_format.space_after=Pt(7)
    p.paragraph_format.keep_with_next=keep
    r=p.add_run(s);r.font.size=Pt(10)
    return p

def hyperlink(p,label,url):
    h=OxmlElement('w:hyperlink');h.set(qn('r:id'),p.part.relate_to(url,RT.HYPERLINK,is_external=True))
    r=OxmlElement('w:r');pr=OxmlElement('w:rPr')
    color=OxmlElement('w:color');color.set(qn('w:val'),'000000');pr.append(color)
    u=OxmlElement('w:u');u.set(qn('w:val'),'single');pr.append(u)
    fonts=OxmlElement('w:rFonts');fonts.set(qn('w:eastAsia'),'宋体');pr.append(fonts)
    size=OxmlElement('w:sz');size.set(qn('w:val'),'20');pr.append(size)
    r.append(pr);t=OxmlElement('w:t');t.text=label;r.append(t);h.append(r);p._p.append(h)

def result_table():
    caption=small_par('表 1  正式载荷测试 G2 组在 2 秒总预算下的合格返回',True)
    rows=[['方法','合格返回次数','返回率'],
          ['历史轨迹检索','87 / 144','60.42%'],
          ['确定性回归','83 / 144','57.64%'],
          ['基础扩散','85 / 144','59.03%'],
          ['物理特征增强扩散','94 / 144','65.28%'],
          ['物理损失扩散','81 / 144','56.25%'],
          ['物理引导扩散','83 / 144','57.64%'],
          ['IHSID 短预算求解','0 / 144','0.00%']]
    table=doc.add_table(rows=len(rows), cols=3)
    table.alignment=WD_TABLE_ALIGNMENT.CENTER
    table.autofit=False
    widths=[3.05,1.7,1.55]
    pr=table._tbl.tblPr
    borders=OxmlElement('w:tblBorders')
    for side in ['top','left','bottom','right','insideH','insideV']:
        b=OxmlElement('w:'+side);b.set(qn('w:val'),'single');b.set(qn('w:sz'),'4');b.set(qn('w:color'),'D9D9D9');borders.append(b)
    pr.append(borders)
    for i,row in enumerate(table.rows):
        trpr=row._tr.get_or_add_trPr()
        if i==0:
            rep=OxmlElement('w:tblHeader');trpr.append(rep)
        cant=OxmlElement('w:cantSplit');trpr.append(cant)
        for j,cell in enumerate(row.cells):
            cell.width=Inches(widths[j]);cell.vertical_alignment=WD_CELL_VERTICAL_ALIGNMENT.CENTER
            cp=cell._tc.get_or_add_tcPr()
            sh=OxmlElement('w:shd');sh.set(qn('w:fill'),'E2E8EE' if i==0 else 'FFFFFF');cp.append(sh)
            mar=OxmlElement('w:tcMar')
            for side,value in [('top',100),('bottom',100),('left',110),('right',110)]:
                item=OxmlElement('w:'+side);item.set(qn('w:w'),str(value));item.set(qn('w:type'),'dxa');mar.append(item)
            cp.append(mar)
            p=cell.paragraphs[0];p.paragraph_format.first_line_indent=Pt(0);p.paragraph_format.space_after=Pt(0);p.paragraph_format.line_spacing=1.15
            p.alignment=WD_ALIGN_PARAGRAPH.LEFT if j==0 else WD_ALIGN_PARAGRAPH.CENTER
            p.paragraph_format.keep_with_next=i<len(rows)-1
            r=p.add_run(rows[i][j]);r.font.size=Pt(11);r.bold=i==0
    small_par('分母为 48 个任务条件 × 3 个种子区组。成功包含本轮允许的后续处理；数值来自 method_group_budget.json，不能当成单候选直接生成率。')

heading_list=[]
pending_page=False
for line in text.splitlines():
    line=line.strip()
    if not line: continue
    if line.startswith('@TITLE '): doc.add_paragraph(line[7:],'Title')
    elif line.startswith('@SUBTITLE '): doc.add_paragraph(line[10:],'Subtitle')
    elif line.startswith('@META '): small_par(line[6:])
    elif line.startswith('@NOTE '): small_par(line[6:])
    elif line=='@PAGE': pending_page=True
    elif line.startswith('# '):
        title=line[2:]
        if pending_page and title in ['交流备答','汇报前材料核对说明']:
            doc.add_page_break()
        pending_page=False
        doc.add_paragraph(title,'Heading 1');heading_list.append(title)
    elif line.startswith('## '): doc.add_paragraph(line[3:],'Heading 2')
    elif line.startswith('@TABLE '): result_table()
    elif line.startswith('@SOURCE '):
        sid,path,description=line[8:].split('|',2)
        p=small_par('')
        p.add_run(sid+'  ')
        hyperlink(p,Path(path).name,Path(path).as_uri())
        p.add_run('  '+description)
    elif line.startswith('@REF '):
        name,url,description=line[5:].split('|',2)
        p=small_par('');hyperlink(p,name,url);p.add_run('。'+description)
    elif line.startswith('[') and line.endswith(']'):
        small_par(line,True)
    else:
        doc.add_paragraph(line)

doc.core_properties.title='基于扩散模型的轨迹规划组会讲稿'
doc.core_properties.subject='对应2026年9月29日PPT的第一人称通俗讲解与交流备答'
doc.core_properties.author='谢振宇'
doc.core_properties.keywords='Stewart 轨迹规划 IHSID 扩散模型 DDPM B样条 UKF'
doc.core_properties.comments='根据用户提供PPT与2026年9月26日读取的项目材料编制。'
doc.save(OUT)
main=text.split('# 交流备答')[0]
summary={'output':str(OUT),'bytes':OUT.stat().st_size,'sha256':hashlib.sha256(OUT.read_bytes()).hexdigest(),
         'paragraphs':len(doc.paragraphs),'tables':len(doc.tables),'main_cjk':len(re.findall(r'[\u4e00-\u9fff]',main)),
         'total_cjk':len(re.findall(r'[\u4e00-\u9fff]',text)),'headings':heading_list}
(WORK/'build_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False,indent=2))
