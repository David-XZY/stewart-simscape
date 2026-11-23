from pathlib import Path
from zipfile import ZipFile
from lxml import etree as E
from PIL import Image
from io import BytesIO
import re, json, hashlib, shutil

BASE = Path(__file__).resolve().parents[1]
SRC = Path(r'D:\Wechat\xwechat_files\wxid_gru35hi5i0ox21_3cd6\temp\RWTemp\2026-11\d60bb657bb46f76d72239ac380a25c6d')
NS = {'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main', 'a':'http://schemas.openxmlformats.org/drawingml/2006/main', 'r':'http://schemas.openxmlformats.org/officeDocument/2006/relationships'}
def text(n): return ''.join(n.xpath('.//w:t/text()',namespaces=NS))
def sha(b): return hashlib.sha256(b).hexdigest()

z=ZipFile(SRC/'main.docx')
root=E.fromstring(z.read('word/document.xml'))
body=list(root.find('w:body',NS))
blocks={i:text(n) for i,n in enumerate(body,1)}
rels={n.get('Id'):n.get('Target') for n in E.fromstring(z.read('word/_rels/document.xml.rels'))}
manifest={'inputs':{n:sha((SRC/n).read_bytes()) for n in ['main.docx','TAES_template.tex','IEEEtaes.cls']},'blocks':[], 'images':[], 'math':[], 'tables':[], 'transformations':[]}
for i,n in enumerate(body,1):
    manifest['blocks'].append({'id':i,'type':E.QName(n).localname,'text':blocks[i]})
shutil.copyfile(SRC/'IEEEtaes.cls',BASE/'IEEEtaes.cls')

# Original image bytes, not screenshots or re-encoded images.
for i,n in enumerate(body,1):
    for rid in n.xpath('.//a:blip/@r:embed',namespaces=NS):
        name='word/'+rels[rid]
        data=z.read(name)
        out='figures/'+Path(name).name
        (BASE/out).write_bytes(data)
        im=Image.open(BytesIO(data))
        manifest['images'].append({'block':i,'source':name,'output':out,'sha256':sha(data),'size':im.size})

for i in [207,221]:
    rows=[]
    for row in body[i-1].findall('w:tr',NS):
        rows.append([{'text':text(c),'span':int((c.xpath('./w:tcPr/w:gridSpan/@w:val',namespaces=NS) or ['1'])[0])} for c in row.findall('w:tc',NS)])
    manifest['tables'].append({'id':i,'rows':rows})

eqnum={}
for i,n in enumerate(body,1):
    instr=n.xpath('.//w:instrText/text()',namespaces=NS)
    if i>=34 and any('SEQ MTEqn' in x for x in instr):
        nums=[int(x.strip()) for x in instr if x.strip().isdigit()]
        assert len(nums)==1,(i,nums)
        eqnum[i]=nums[0]

def esc(t):
    # Keep spelling, punctuation and all Unicode source characters.
    m={'&':r'\&','%':r'\%','#':r'\#','_':r'\_','{':r'\{','}':r'\}','~':r'\textasciitilde{}','^':r'\textasciicircum{}','\\':r'\textbackslash{}','"':r'\sourcequote{}',"'":r'\textquotesingle{}'}
    t=''.join(m.get(c,c) for c in t)
    # Preserve literal URL text, including the source's malformed DOI, while allowing wrapping.
    t=re.sub(r'https?://[^\s]+',lambda m:r'\url{'+m[0]+'}',t)
    return t

used=set()
def mark(ids):
    used.update(ids)
    return '% SOURCE_BLOCKS: '+','.join(map(str,ids))+'\n'

def math_open(t):
    state=None
    for m in re.finditer(r'\\\[|\\\]|\$',t):
        tok=m[0]
        if tok=='\\[': assert state is None; state='display'
        elif tok=='\\]': assert state=='display';state=None
        else: state=None if state=='inline' else 'inline'
    return state

# Optional layout overrides are written independently of the source extraction.
layout_path=BASE/'_work'/'math_layout.json'
layouts=json.loads(layout_path.read_text('utf-8')) if layout_path.exists() else {}
mathid=0
def convert(t,ids,number=None):
    global mathid
    out=[];pos=0
    pattern=re.compile(r'\\\[(.*?)\\\]|\$(.*?)\$',re.S)
    matches=list(pattern.finditer(t))
    for mi,m in enumerate(matches):
        plain=t[pos:m.start()]
        # Table numerals are template labels; the sentence itself stays intact.
        for arab,roman in [('1','I'),('2','II')]:
            plain=plain.replace('Table '+arab,'Table '+roman)
        out.append(esc(plain))
        mathid+=1
        raw=m[1] if m[1] is not None else m[2]
        isdisplay=m[1] is not None or r'\begin{array}' in raw or (number is not None and len(matches)==1 and not t[:m.start()].strip() and not t[m.end():].strip())
        num=number if mi==len(matches)-1 else None
        layout=layouts.get(str(mathid),{})
        formatted=layout.get('latex',raw)
        # \hfill and whitespace-only \text groups are MathType alignment controls.
        formatted=formatted.replace(r'\hfill','')
        formatted=re.sub(r'\{\\text\{\s*\}\}', '', formatted)
        punctuation=t[m.end():m.end()+1] if isdisplay and t[m.end():m.end()+1] in [',','.',';'] else ''
        manifest['math'].append({'id':mathid,'blocks':ids,'source':raw,'latex':formatted,'display':isdisplay,'number':num,'wide':layout.get('wide',False),'trailing_punctuation':punctuation})
        if isdisplay:
            env='equation' if num is not None else 'equation*'
            code='\n'+r'\begin{'+env+'}\n'+formatted+(r'\text{'+punctuation+'}' if punctuation else '')+'\n'
            if num is not None:code+=r'\tag{'+str(num)+'}\n'
            code+=r'\end{'+env+'}\n'
            code='% MATH_ID: '+str(mathid)+'\n'+code
            # Measure using exactly the final math font, without inserting output.
            measure='\\begingroup\\setbox0=\\hbox{$\\displaystyle '+formatted+'$}\\typeout{MATHWIDTH:'+str(mathid)+':\\the\\wd0}\\endgroup\n'
            if layout.get('wide'):
                code='\n\\begin{figure*}[!t]\n'+measure+code+'\\end{figure*}\n'
            else: code=measure+code
            out.append(code)
        else:
            if layout.get('display'):
                out.append('\n\\[\n'+formatted+'\n\\]\n')
            else: out.append('$'+formatted+'$')
        pos=m.end()+len(punctuation)
    tail=t[pos:]
    for arab,roman in [('1','I'),('2','II')]:tail=tail.replace('Table '+arab,'Table '+roman)
    out.append(esc(tail))
    return ''.join(out)

preamble=r'''% Compile with XeLaTeX (two passes). All images use relative paths.
% Content authority: main.docx. Layout authority: supplied TAES_template.tex / IEEEtaes.cls.
\RequirePackage[OT1]{fontenc}
\documentclass{IEEEtaes}
\usepackage{color,array,amsthm}
\usepackage{graphicx}
\usepackage{amsmath,amssymb}
\usepackage{newunicodechar}
\usepackage{placeins}
\usepackage{textcomp}
% Straight quotation marks in the source stay straight; OT1's bare double
% quote slot is a closing typographic quote, so use the same family in T1 locally.
\protected\def\sourcequote{{\fontencoding{T1}\selectfont\textquotedbl}}
% Keep the class's original text and math fonts. Only unsupported Unicode
% characters use a local glyph, without changing the surrounding font metrics.
\makeatletter
\newcommand{\sourceglyph}[2]{{\font\sourcefont="#1" at \f@size pt\relax\sourcefont\char"#2\relax}}
% After the final full-width float page, the legacy class can retain the
% previous reduced column height. Restore its own type-area dimensions.
\newcommand{\resumeTAEScolumns}{\global\@colht\textheight\global\@colroom\textheight\global\vsize\textheight}
\makeatother
\newunicodechar{à}{\`a}
\newunicodechar{á}{\'a}
\newunicodechar{ý}{\'y}
\newunicodechar{Λ}{\sourceglyph{Times New Roman}{039B}}
\newunicodechar{∆}{\sourceglyph{Times New Roman}{2206}}
\newunicodechar{，}{\sourceglyph{SimSun}{FF0C}}
\newunicodechar{‐}{\sourceglyph{Times New Roman}{2010}}
\newunicodechar{–}{\textendash}
\newunicodechar{—}{\textemdash}
\newunicodechar{’}{\textquoteright}
\newunicodechar{​}{\hspace{0pt}}
% XeTeX must receive the class's final paper dimensions (the class contains an
% earlier legacy papersize special). This matches the supplied template in pdfLaTeX.
\AtBeginDvi{\special{papersize=\the\paperwidth,\the\paperheight}}
% MathType's user-style marker is present in the Word source. Preserve its
% argument verbatim: the two consecutive equals signs in equation (20) are original.
\def\user1#1{#1}
% The class expands affiliation fields with edef; protect the original markers.
\protected\def\sourcemark#1{\textsuperscript{#1}}
\jvol{}
\jnum{}
\jmonth{}
\paper{}
\pubyear{}
\doiinfo{}
\newtheorem{theorem}{Theorem}
\newtheorem{lemma}{Lemma}
\setcounter{page}{1}
\begin{document}
'''
out=[preamble]
out+=[mark([3])+r'\title{'+esc(blocks[3])+'}\n']
out+=[mark([4,5,6])+r'''\author{XXX\sourcemark{a} ,}
\member{}
\affil{\sourcemark{a}XXX}
\author{XXX\sourcemark{b,*}}
\member{}
\affil{\sourcemark{b}XXX}
''']
out+=[mark([9])+r'\receiveddate{'+esc(blocks[9])+'}\n',mark([13])+r'\corresp{'+esc(blocks[13])+'}\n']
out+=[r'\authoraddress{}'+'\n'+r'\editor{}'+'\n'+r'\supplementary{}'+'\n']
out+=[r'\markboth{XXX, XXX}{'+esc(blocks[3])+'}\n'+r'\maketitle'+'\n']
out+=[mark([10,11])+r'\begin{abstract}'+convert(blocks[11],[11])+r'\end{abstract}'+'\n']
out+=[mark([12])+r'\begin{IEEEkeywords}'+convert(blocks[12].removeprefix('Keywords: '),[12])+r'\end{IEEEkeywords}'+'\n']

headings={15:1,25:1,26:2,32:2,33:3,89:3,105:1,106:2,121:2,138:3,160:3,177:2,201:1,222:1}
figs={30:(31,1,'image3.png'),136:(137,2,'image4.png'),203:(204,3,'image5.png'),209:(210,4,'image6.png'),212:(213,5,'image7.png'),217:(218,8,'image10.png')}
# Large multi-panel figures retain readable original labels at page width.
widefigs={1,2,3,5,8}
def figure(b,cap,num,img):
    title=re.sub(r'^Fig\.\s*\d+\s*','',blocks[cap])
    wide=num in widefigs
    env='figure*' if wide else 'figure'
    width=r'\textwidth' if wide else r'\columnwidth'
    return mark([b,cap])+f'\\begin{{{env}}}[!t]\n\\centering\n\\includegraphics[width={width}]{{figures/{img}}}\n\\caption{{'+esc(title)+'}\n\\label{fig:'+str(num)+'}\n'+f'\\end{{{env}}}\n'

def table1():
    data=manifest['tables'][0]['rows']
    title=re.sub(r'^Table\s*1\s*','',blocks[206])
    s=mark([206,207])+r'\begin{table}[t]'+'\n'+r'\caption{'+esc(title)+'}\n'+r'\label{tab:1}\centering\tablefont'+'\n'
    s+=r'\begin{tabular}{@{}p{0.31\columnwidth}p{0.62\columnwidth}@{}}\hline'+'\n'
    for j,row in enumerate(data):
        s+=' & '.join(convert(c['text'],[207]) for c in row)+r' \\'+'\n'
        if j==0:s+=r'\hline'+'\n'
    return s+r'\hline\end{tabular}'+'\n'+r'\end{table}'+'\n'

def table2():
    data=manifest['tables'][1]['rows']
    title=re.sub(r'^Table\s*2\s*','',blocks[220])
    s=mark([220,221])+r'\begin{table*}[t]'+'\n'+r'\caption{'+esc(title)+'}\n'+r'\label{tab:2}\centering\tablefont'+'\n'
    s+=r'\begin{tabular*}{\textwidth}{@{\extracolsep{\fill}}p{0.22\textwidth}cccccc@{}}\hline'+'\n'
    for j,row in enumerate(data):
        cells=[]
        for c in row:
            t=convert(c['text'],[221])
            if c['span']>1:t=r'\multicolumn{'+str(c['span'])+'}{c}{'+t+'}'
            cells.append(t)
        s+=' & '.join(cells)+r' \\'+'\n'
        if j==1:s+=r'\hline'+'\n'
    return s+r'\hline\end{tabular*}'+'\n'+r'\end{table*}'+'\n'

def pairfigure():
    row=body[214].find('w:tr',NS)
    pair=mark([215])+r'\begin{figure*}[!t]\centering'+'\n'
    for k,cell in enumerate(row.findall('w:tc',NS),6):
        cap=re.sub(r'^Fig\.\s*\d+\s*','',text(cell))
        pair+=r'\begin{minipage}[t]{0.48\textwidth}\vspace{0pt}\centering'+'\n'+r'\includegraphics[width=\linewidth]{figures/image'+str(k+2)+'.png}\n'+r'\caption{'+esc(cap)+'}\n'+r'\label{fig:'+str(k)+'}\n'+r'\end{minipage}'+(r'\hfill%' if k==6 else '')+'\n'
    return pair+r'\end{figure*}'+'\n'

i=15
while i<=229:
    if i in used:i+=1;continue
    # Queue floats at the start of their first referring paragraph. The prose
    # order remains exact; figures can then occupy a nearby column/page top.
    if i==27:out.append(figure(30,*figs[30]))
    if i==122:out.append(figure(136,*figs[136]))
    if i==202:out.append(figure(203,*figs[203]))
    if i==205:out.extend([table1(),figure(209,*figs[209])])
    if i==211:out.append(figure(212,*figs[212]))
    if i==214:out.extend([pairfigure(),figure(217,*figs[217])])
    if i==219:out.append(table2())
    if i in figs:
        out.append(figure(i,*figs[i]));i+=2;continue
    if i==206:out.append(table1());i=208;continue
    if i==220:out.append(table2());i=222;continue
    if i==215:
        out.append(pairfigure());i+=1;continue
    if i in headings:
        title=re.sub(r'^\d+(?:\.\d+)*\.?\s*','',blocks[i]).rstrip()
        command={1:'section',2:'subsection',3:'subsubsection'}[headings[i]]
        if i in [105,177,201]:out.append(r'\FloatBarrier'+'\n')
        if i==222:out.append(r'\clearpage\resumeTAEScolumns'+'\n')
        title_tex=esc(title)
        if headings[i]==1:title_tex=r'\MakeUppercase{'+title_tex+'}'
        out.append(mark([i])+'\\'+command+'{'+title_tex+'}\n');i+=1;continue
    if i==228:
        out.append(mark([i])+r'\section*{\MakeUppercase{'+esc(blocks[i])+'}}\n');i+=1;continue
    if not blocks[i].strip():used.add(i);i+=1;continue
    ids=[i];t=blocks[i]
    while math_open(t):
        i+=1;ids.append(i);t+='\n'+blocks[i]
    out.append(mark(ids)+convert(t,ids,eqnum.get(i))+'\n\n')
    i+=1

out.append(mark([230])+r'\bibsection*{\MakeUppercase{'+esc(blocks[230])+'}}\n'+r'\def\refname{}'+'\n'+r'\IEEEtriggeratref{32}'+'\n'+r'\begin{thebibliography}{[39]}'+'\n')
for i in range(231,270):
    t=blocks[i];m=re.match(r'^\[(\d+)\](.*)$',t,re.S);assert m
    out.append(mark([i])+r'\bibitem{ref'+m[1]+'}'+convert(m[2],[i])+'\n')
out.append(r'\end{thebibliography}'+'\n'+r'\end{document}'+'\n')
tex='\n'.join(out)
(BASE/'main_taes.tex').write_text(tex,encoding='utf-8')
manifest['unmapped_nonempty_blocks']=[i for i,t in blocks.items() if t.strip() and i not in used]
assert not manifest['unmapped_nonempty_blocks'],manifest['unmapped_nonempty_blocks']
assert len(manifest['images'])==8
assert len(manifest['math'])==135,len(manifest['math'])
manifest['transformations']=['Old journal headers/logos replaced by TAES layout.','Section numbering and table labels use TAES style; Table 1/2 prose references become Table I/II.','Keywords label is rendered as the template Index Terms label.','Word MathType source is rendered as mathematics. Thirty equation number markers retained as tags; empty prose references remain empty.','Only formula alignment whitespace, line breaks and environments may change.','No bibliography normalization or language correction.']
(BASE/'_work'/'source_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'blocks':len(blocks),'mapped_nonempty':len([i for i in used if blocks[i].strip()]),'math':len(manifest['math']),'images':len(manifest['images']),'equation_tags':len(eqnum),'tables':len(manifest['tables']),'source_sha256':manifest['inputs']},ensure_ascii=False))
