"""Independent source-to-TeX content and artifact checks; no manuscript edits."""
from pathlib import Path
from zipfile import ZipFile
from lxml import etree as E
from PIL import Image
from io import BytesIO
import json,re,hashlib,unicodedata
import pymupdf as fitz

BASE=Path(__file__).resolve().parents[1]
SRC=Path(r'D:\Wechat\xwechat_files\wxid_gru35hi5i0ox21_3cd6\temp\RWTemp\2026-11\d60bb657bb46f76d72239ac380a25c6d')
NS={'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main'}
with ZipFile(SRC/'main.docx') as z:
    root=E.fromstring(z.read('word/document.xml'))
    original={i:''.join(n.xpath('.//w:t/text()',namespaces=NS)) for i,n in enumerate(root.find('w:body',NS),1)}
m=json.loads((BASE/'_work/source_manifest.json').read_text('utf8'))
tex=(BASE/'main_taes.tex').read_text('utf8')
issues=[]
def norm(s): return re.sub(r'\s+','',s.replace('\u200b',''))

def group(s,pos):
    while pos<len(s) and s[pos].isspace():pos+=1
    assert s[pos]=='{',(s[pos:pos+50],pos)
    start=pos+1;depth=1;pos+=1
    while depth:
        if s[pos]=='\\':pos+=2;continue
        if s[pos]=='{':depth+=1
        elif s[pos]=='}':depth-=1
        pos+=1
    return s[start:pos-1],pos

def prose_source(s):
    return re.sub(r'\\\[.*?\\\]|\$.*?\$','',s,flags=re.S)

def prose_tex(s,fields=None):
    # Remove the non-rendered width measurement, which contains a second copy.
    s=re.sub(r'\\begingroup\\setbox0=.*?\\endgroup','',s,flags=re.S)
    s=re.sub(r'(?<!\\)%[^\n]*','',s)
    # Punctuation moved from after a display into its final line is prose content.
    def remove_display(mm):
        pp=re.findall(r'\\text\{([,.;])\}',mm[0])
        return ''.join(pp)
    s=re.sub(r'\\begin\{equation\*?\}.*?\\end\{equation\*?\}|\\\[.*?\\\]',remove_display,s,flags=re.S)
    s=re.sub(r'\$.*?\$','',s,flags=re.S)
    s=s.replace(r'\def\refname{}','')
    out=[];i=0
    keep={'title','author','affil','receiveddate','corresp','caption','section','subsection','subsubsection','bibsection','MakeUppercase','sourcemark','textsuperscript','textsubscript','emph','textbf','textit','textrm','url'}
    skip={'label':1,'member':1,'markboth':2,'includegraphics':1,'bibitem':1,'authoraddress':1,'editor':1,'supplementary':1,'vspace':1,'hspace':1,'setcounter':2,'IEEEtriggeratref':1}
    while i<len(s):
        c=s[i]
        if c=='\\':
            mm=re.match(r'\\([A-Za-z@]+\*?|.)',s[i:],re.S);cmd=mm[1];i+=len(mm[0]);base=cmd.rstrip('*')
            if cmd in ['&','%','#','_','{','}']:
                out.append(cmd);continue
            if cmd=='\\':out.append(' ');continue
            if cmd in ['sourcequote','textquotesingle']:
                out.append('"' if cmd=='sourcequote' else "'");continue
            if cmd in ['textasciitilde','textasciicircum','textbackslash']:
                out.append({'textasciitilde':'~','textasciicircum':'^','textbackslash':'\\'}[cmd]);continue
            if base in ['begin','end']:
                env,i=group(s,i)
                if base=='begin':
                    while i<len(s) and s[i]=='[':i=s.index(']',i)+1
                    n={'tabular':1,'tabular*':2,'minipage':1,'thebibliography':1}.get(env,0)
                    for _ in range(n):_,i=group(s,i)
                continue
            if base in keep:
                arg,i=group(s,i);val=prose_tex(arg)
                if fields is not None:fields.setdefault(base,[]).append(val)
                out.append(val);continue
            if base in skip:
                while i<len(s) and s[i]=='[':i=s.index(']',i)+1
                for _ in range(skip[base]):_,i=group(s,i)
                continue
            if base=='multicolumn':
                _,i=group(s,i);_,i=group(s,i);arg,i=group(s,i);out.append(prose_tex(arg));continue
            if base in ['centering','tablefont','hline','hfill','FloatBarrier','maketitle','noindent','relax','par','clearpage','resumeTAEScolumns']:continue
            issues.append('Unhandled prose command: '+cmd)
        elif c in '{}&':i+=1
        else:out.append(c);i+=1
    return ''.join(out)

headers={15,25,26,32,33,89,105,106,121,138,160,177,201,222}
chunks=list(re.finditer(r'^% SOURCE_BLOCKS: ([\d,]+)\n',tex,re.M))
text_checks=[];seen=[]
for idx,chunk in enumerate(chunks):
    ids=[int(x) for x in chunk[1].split(',')];seen.extend(ids)
    output=tex[chunk.end():chunks[idx+1].start() if idx+1<len(chunks) else len(tex)]
    fields={};actual=prose_tex(output,fields)
    if ids==[4,5,6]:
        expected_author=norm(original[4]);actual_author=norm(''.join(fields['author']))
        expected_affil=norm(original[5]+original[6]);actual_affil=norm(''.join(fields['affil']))
        passed=expected_author==actual_author and expected_affil==actual_affil
        text_checks.append({'blocks':ids,'pass':passed,'kind':'author/affiliation fields'})
        if not passed:issues.append('Author fields mismatch')
        continue
    source=[]
    for b in ids:
        s=original[b]
        if b==10:s=''
        if b==12:s=s.removeprefix('Keywords: ')
        if b in headers:s=re.sub(r'^\d+(?:\.\d+)*\.?\s*','',s)
        if b in [31,137,204,210,213,218,215]:s=re.sub(r'Fig\.\s*\d+\s*','',s)
        if b in [206,220]:s=re.sub(r'^Table\s*\d+\s*','',s)
        if 231<=b<=269:s=re.sub(r'^\[\d+\]','',s)
        s=s.replace('Table 1','Table I').replace('Table 2','Table II')
        source.append(s)
    expected=prose_source('\n'.join(source))
    passed=norm(actual)==norm(expected)
    text_checks.append({'blocks':ids,'pass':passed,'kind':'nonmath characters'})
    if not passed:
        import difflib
        diff=list(difflib.ndiff([norm(expected)],[norm(actual)]))
        issues.append({'blocks':ids,'difference':diff})

def math_norm(s):
    s=s.replace(r'\hfill','')
    s=re.sub(r'\{\\text\{\s*\}\}','',s)
    s=re.sub(r'\\setlength\{\\arraycolsep\}\{[^}]*\}','',s)
    s=re.sub(r'\\(?:begin|end)\{gathered\}','',s)
    # Alignment breaks and redundant group wrappers carry no mathematical tokens.
    s=s.replace('\\\\','')
    return re.sub(r'[\s{}]','',s)

math_checks=[]
for q in m['math']:
    passed=math_norm(q['source'])==math_norm(q['latex'])
    # Array row and column boundaries must also agree, not just token order.
    def arrays(s):
        return [re.sub(r'\s+','',x) for x in re.findall(r'\\begin\{array\}(.*?)\\end\{array\}',s,re.S)]
    passed=passed and arrays(q['source'])==arrays(q['latex'])
    math_checks.append({'id':q['id'],'blocks':q['blocks'],'number':q['number'],'pass':passed})
    if not passed:issues.append('Math mismatch '+str(q['id']))

image_checks=[]
for q in m['images']:
    digest=hashlib.sha256((BASE/q['output']).read_bytes()).hexdigest()
    image_checks.append({'file':q['output'],'pass':digest==q['sha256'],'sha256':digest})
    if digest!=q['sha256']:issues.append('Image bytes mismatch '+q['output'])

pdf=fitz.open(BASE/'_work/main_taes.pdf')
template=fitz.open(BASE/'_work/template_reference.pdf')
pdfimages=[r for p in pdf for r in p.get_images(full=True)]
pdf_image_checks=[]
for q in m['images']:
    matches=[r for r in pdfimages if list(r[2:4])==q['size']]
    assert len(matches)==1,(q['output'],len(matches))
    pix=fitz.Pixmap(pdf,matches[0][0]);im=Image.open(BASE/q['output']).convert('RGB')
    if pix.n!=3:pix=fitz.Pixmap(fitz.csRGB,pix)
    passed=pix.samples==im.tobytes()
    pdf_image_checks.append({'file':q['output'],'pixel_identical':passed})
    if not passed:issues.append('PDF image pixels mismatch '+q['output'])

log=(BASE/'_work/main_taes.log').read_text(errors='replace')
warnings={k:re.findall(v,log) for k,v in {'overfull':r'Overfull[^\n]*','missing_glyphs':r'Missing character[^\n]*','font_substitution':r'LaTeX Font Warning[^\n]*','undefined_reference':r'(?:undefined references|Reference[^\n]*undefined|Citation[^\n]*undefined)','errors':r'^![^\n]*'}.items()}
assert all(not v for v in warnings.values()),warnings
assert pdf[0].rect==template[0].rect
bodyfonts=lambda doc: sorted({(sp['font'],round(sp['size'],2)) for pg in doc for bl in pg.get_text('dict')['blocks'] if 'lines' in bl for ln in bl['lines'] for sp in ln['spans'] if sp['font'].startswith('Nimbus')})
template_fonts=bodyfonts(template);out_fonts=bodyfonts(pdf)
extra=[x for x in out_fonts if x[0] not in {f for f,s in template_fonts} or x[1] not in {s for f,s in template_fonts}]
assert not extra,extra
source_unchanged={n:hashlib.sha256((SRC/n).read_bytes()).hexdigest()==h for n,h in m['inputs'].items()}
assert all(source_unchanged.values())
assert hashlib.sha256((BASE/'IEEEtaes.cls').read_bytes()).hexdigest()==m['inputs']['IEEEtaes.cls']

result={'pass':not issues,'pages':len(pdf),'source_files_unchanged':source_unchanged,'class_byte_identical':True,'paper_points':list(pdf[0].rect),'template_paper_points':list(template[0].rect),'text_block_checks':text_checks,'math_checks':math_checks,'image_checks':image_checks,'pdf_image_checks':pdf_image_checks,'font_sizes_match_template':not extra,'warnings':warnings,'issues':issues}
(BASE/'_work/verification_report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({'pass':result['pass'],'pages':len(pdf),'text_checks':len(text_checks),'math_checks':len(math_checks),'image_checks':len(image_checks),'pdf_image_pixels':[q['pixel_identical'] for q in pdf_image_checks],'issues':issues},ensure_ascii=False))
