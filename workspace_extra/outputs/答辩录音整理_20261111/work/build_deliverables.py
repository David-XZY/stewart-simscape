import json, pathlib, re, html, hashlib, zipfile
from transcribe_local import stamp

ROOT=pathlib.Path(__file__).resolve().parent.parent
WORK=ROOT/'work'
def read_rows(name):
    return [json.loads(s) for s in (WORK/name).read_text('utf-8').splitlines()]
q4=read_rows('pass1_q4.jsonl'); q8=read_rows('pass2_q8.jsonl'); sv=read_rows('sensevoice.jsonl')
assert len(q4)==len(q8)==len(sv)==176
assert [r['index'] for r in q4]==list(range(176))
overrides={}
for line in (WORK/'校订覆盖.txt').read_text('utf-8').splitlines():
    i,t=line.split('|',1)
    assert int(i) not in overrides
    overrides[int(i)]=t
assert set(overrides)==set(range(176))
for a,b in zip(q4,q4[1:]): assert abs(a['end']-b['start'])<1e-5
assert q4[0]['start']==0 and abs(q4[-1]['end']-3693.4613125)<0.01

sections={0:'开场与汇报安排',2:'开题汇报',35:'第一轮评议与问答',86:'第二轮评议与问答',162:'结尾意见与现场安排'}
intro='''完整时间轴转写·校订稿
课题：多元扰动下多智能体协同鲁棒性分析（题目由用户确认）
录音：2026年11月11日 10点31分.m4a
总时长：01:01:33；全程分为176个连续时间段。

阅读说明
1. 使用本地面试狗的 Fun-ASR-Nano 引擎，完成 Q4 初转及 Q8 术语辅助复转，并用本地 SenseVoice 交叉对照；另对含混段落进行了短分段复转。音频未上传飞书或其他云端转写服务。
2. 本稿是根据多份识别结果、前后语境和手写笔记逐段校订的口语转写稿，不是已经逐字听辨验真的速记稿。保留汇报、问答和结尾内容；清除机器重复循环及切段重叠，部分无实义口头重复作合并。不能保证每个字均准确。
3. 【】内为整理者的待核、暂读或补明说明，不是录音原话。名字、低声交谈、公式符号、具体数值仍有不能确认之处；不以学术常识补造缺失发言。每段起止时间为音频切段边界，不是逐字对齐时间。
4. 问答中的“——”仅表示语境上明显的话轮切换；没有用声纹识别确定个人身份。多人插话仍可能在同一段内。图片的“马”“刘”只作为笔记分组，不据此给整段录音实名归属。
5. 用户确认的题目与开场机器识别到的题目表述略有不同，已在00:00:41段保留说明；姓名未获得用户确认。
6. 以下保留现场观点，包括可能需要进一步商榷的技术表述；未独立审核答辩报告的公式及研究结论。
'''
parts=[intro]
records=[]
for row in q4:
    i=row['index'];t=overrides[i]
    if i in sections: parts.append('\n'+sections[i]+'\n')
    parts.append(f"[{stamp(row['start'])}—{stamp(row['end'])}]\n{t}\n")
    records.append(dict(index=i,start=row['start'],end=row['end'],text=t,uncertain=('【' in t)))
(ROOT/'01_完整录音转写_校订稿.txt').write_text('\n'.join(parts),'utf-8-sig')
(WORK/'corrected.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),'utf-8')

raw=[]
for i,r in enumerate(q4):
    raw.append(f"[{stamp(r['start'])}—{stamp(r['end'])}]\nFun-ASR-Nano Q4：{r['text']}\nFun-ASR-Nano Q8（术语辅助）：{q8[i]['text']}\nSenseVoice：{sv[i]['text']}\n")
(ROOT/'05_机器识别原稿对照_未经校订.txt').write_text('机器识别原稿对照（包含误识别、漏词及重复循环，仅用于溯源）\n\n'+'\n'.join(raw),'utf-8-sig')
uncertain=[r for r in records if r['uncertain']]
pending=['待核字句索引','这里只列出校订稿中的显式疑点。没有标记不等于已通过原音逐字验真。','']
for r in uncertain:
    pending.append(f"[{stamp(r['start'])}—{stamp(r['end'])}] "+'；'.join(re.findall(r'【(.*?)】',r['text'])))
(ROOT/'04_待核字句索引.txt').write_text('\n'.join(pending),'utf-8-sig')

E=html.escape
audio=pathlib.Path(r'D:\QQfiles\Document\2026年11月11日 10点31分.m4a').as_uri()
cards=[]
for r in records:
    i=r['index']
    if i in sections: cards.append(f'<h3>{E(sections[i])}</h3>')
    marked=re.sub(r'(【.*?】)',r'<mark>\1</mark>',E(r['text']))
    variants=''.join(f'<p><b>{label}</b> {E(data[i]["text"])}</p>' for label,data in [('Q4 初转',q4),('Q8 术语复转',q8),('SenseVoice 对照',sv)])
    cards.append(f'<article class="segment" data-uncertain="{int(r["uncertain"])}" id="s{i}"><button class="timestamp" data-time="{r["start"]}">{stamp(r["start"])}—{stamp(r["end"])}</button><p>{marked}</p><details><summary>查看机器原稿对照</summary>{variants}</details></article>')
notes=(ROOT/'02_手写笔记_录音核对版.txt').read_text('utf-8')
questions=(ROOT/'03_录音问题与修改意见.txt').read_text('utf-8')
def document_text(text):
    return ''.join(f'<p>{E(p).replace(chr(10),"<br>")}</p>' for p in text.split('\n\n') if p.strip())
page='''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>答辩录音整理 · 2026年11月11日</title><style>
:root{--ink:#23313c;--accent:#126c77;--line:#dce4e7}*{box-sizing:border-box}body{margin:0;color:var(--ink);background:#f3f6f7;font:16px/1.85 "Microsoft YaHei",sans-serif}main{max-width:1040px;margin:auto;padding:36px 24px 80px}header{background:#fff;border-top:5px solid var(--accent);padding:30px 34px;border-radius:8px}h1{font-size:30px;line-height:1.4;margin:8px 0}h2{font-size:25px;margin:0 0 24px}h3{font-size:20px;margin:30px 0 14px;color:var(--accent)}.eyebrow{font-size:13px;letter-spacing:1px;color:var(--accent)}.muted{color:#61727c;font-size:14px}.note{background:#edf4f5;border-left:3px solid var(--accent);padding:14px 18px;font-size:14px}nav{display:flex;gap:12px;flex-wrap:wrap;margin:20px 0}a{color:var(--accent)}nav a{background:white;border:1px solid var(--line);border-radius:6px;padding:8px 16px;text-decoration:none}.player{position:sticky;top:0;z-index:5;background:#fffffff5;border:1px solid var(--line);border-radius:7px;padding:12px 18px;box-shadow:0 3px 12px #23313c12}audio{width:100%;height:38px}.player small{display:block;color:#61727c}.block{background:#fff;border:1px solid var(--line);border-radius:8px;padding:30px 34px;margin-top:24px;scroll-margin-top:145px}.tools{display:flex;gap:16px;flex-wrap:wrap;margin-bottom:20px;align-items:center}.tools input[type=search]{font:inherit;padding:8px 12px;border:1px solid #bac9ce;border-radius:5px;width:330px;max-width:100%}.segment{padding:18px 0;border-bottom:1px solid var(--line);scroll-margin-top:145px}.segment p{margin:9px 0 4px;white-space:pre-wrap;overflow-wrap:anywhere}.timestamp{color:var(--accent);border:0;background:#eaf4f4;border-radius:4px;padding:4px 9px;font:600 14px/1.5 Consolas,monospace;cursor:pointer}mark{background:#fff0ce;color:#805709}details{margin-top:9px;color:#5d6c74;font-size:13px}details p{line-height:1.7}summary{cursor:pointer}.document p{margin:0 0 22px}.hidden{display:none}footer{padding-top:25px;font-size:13px;color:#65747d}button:focus,a:focus{outline:2px solid #3b929d;outline-offset:3px}@media(max-width:650px){main{padding:16px 12px}header,.block{padding:22px 18px}h1{font-size:24px}h2{font-size:22px}}@media print{body{background:#fff;font-size:10.5pt}main{max-width:none;margin:0;padding:0}nav,.player,.tools,details,footer{display:none}header,.block{border:0;padding:0;margin:0 0 20px}.block{break-before:page}.segment{break-inside:avoid}mark{background:#fff;color:#444}.timestamp{background:none;padding:0}.hidden{display:block}}
</style></head><body><main><header><div class="eyebrow">2026.11.11 · 硕士开题答辩</div><h1>多元扰动下多智能体协同鲁棒性分析</h1><p class="muted">录音 01:01:33 · 176 个连续时间段 · 24 条手写笔记 · 28 项学术问题</p><div class="note">本页包含完整时间轴校订稿、手写笔记电子版及问题清单。黄色标记为待核或整理说明。依据本地多模型识别与上下文逐段校订，尚不能视为逐字听辨验真的记录；名字、公式与低声插话的疑点均保留。点击时间可回听原录音。</div></header>
<nav><a href="#transcript">完整转写</a><a href="#notes">手写笔记</a><a href="#questions">问题清单</a><a href="04_待核字句索引.txt">待核索引</a></nav>
<div class="player"><audio id="audio" controls preload="metadata" src="AUDIO_URL"></audio><small id="audioStatus">原录音引用本机 D:/QQfiles/Document 中的文件。若移动或删除原文件，回听将不可用。</small></div>
<section class="block" id="transcript"><h2>完整录音转写 · 校订稿</h2><details><summary>转写与校订说明</summary><div class="document">INTRO</div></details><div class="tools"><input type="search" id="search" placeholder="搜索校订稿中的词句" aria-label="搜索词句"><label><input type="checkbox" id="pending"> 只看待核段落</label><span id="count" class="muted"></span></div>TRANSCRIPT</section>
<section class="block document" id="notes"><h2>手写笔记电子版</h2>NOTES</section><section class="block document" id="questions"><h2>录音问题与修改意见</h2>QUESTIONS</section>
<footer>原始录音与图片未修改。整理结果及机器识别原稿对照均保存在同一文件夹。模型来源：<a href="https://huggingface.co/FunAudioLLM/Fun-ASR-Nano-GGUF/tree/main">FunAudioLLM 官方 GGUF 模型仓库</a>。</footer></main>
<script>const audio=document.getElementById('audio');document.querySelectorAll('[data-time]').forEach(b=>b.addEventListener('click',()=>{audio.currentTime=Number(b.dataset.time);audio.play().catch(()=>{document.getElementById('audioStatus').textContent='请先点击播放按钮；若仍无法播放，请确认原录音路径未改变。'})}));const search=document.getElementById('search'),pending=document.getElementById('pending'),segments=[...document.querySelectorAll('.segment')];function filter(){let n=0;for(const e of segments){const match=e.querySelector('p').textContent.toLowerCase().includes(search.value.toLowerCase())&&(!pending.checked||e.dataset.uncertain==='1');e.classList.toggle('hidden',!match);if(match)n++}document.getElementById('count').textContent=n+' / '+segments.length+' 段'}search.addEventListener('input',filter);pending.addEventListener('change',filter);audio.addEventListener('error',()=>{document.getElementById('audioStatus').textContent='原音频未能加载。请确认 D:/QQfiles/Document 下的原始 m4a 文件仍存在，或直接用播放器打开。'});filter();</script></body></html>'''
page=page.replace('AUDIO_URL',E(audio,quote=True)).replace('INTRO',document_text(intro)).replace('TRANSCRIPT','\n'.join(cards)).replace('NOTES',document_text(notes)).replace('QUESTIONS',document_text(questions))
assert page.count('class="segment"')==176
assert all(f'id="s{i}"' in page for i in range(176))
assert 'AUDIO_URL' not in page and 'TRANSCRIPT' not in page
(ROOT/'打开这里_答辩整理.html').write_text(page,'utf-8')

method=f'''处理与校订记录

源文件：D:/QQfiles/Document/2026年11月11日 10点31分.m4a
标题依据：用户于本次对话确认“多元扰动下多智能体协同鲁棒性分析”。
全程覆盖：0.00—3693.4613125秒，176个连续段，无跳段。
识别路径：本地 InterviewCopilot 的 Fun-ASR-Nano worker；F16 编码器；Q4 初转及 Q8 术语辅助复转；SenseVoice Q8 全程交叉参考。
Q8 来源：https://huggingface.co/FunAudioLLM/Fun-ASR-Nano-GGUF/tree/main
Q8 SHA-256：819f385dc0e035dccc3d9e7edaf6b7b044b8ba7ace63cbcbf84c7e397eecbf27（已核验）
Q8 高位宽不能保证每段优于Q4；实际出现漏词、提前停止和循环，校订时未机械采用任一单一结果。
额外核对：46个原始段拆为92个较短段复转；另测试4段增益放大音频，未把放大结果无条件当成更准确版本。
全部176段逐段审阅并生成校订记录。{len(uncertain)}段包含至少一个显式整理／待核标记；这包括人名、数学符号、含混语句及少数语境补明，不是字错误率指标。
重点校订术语：多智能体、集总扰动、匹配／不匹配扰动、线性扩张状态观测器、LESO、ISS耗散条件、LMI、H∞、渐近稳定、舒尔补、Young不等式、李雅普诺夫函数、拉普拉斯矩阵、利普希茨条件、YALMIP、CVX。
质量边界：仅基于多次本地识别、上下文和照片校订，没有人工听辨真值、论文或PPT原文，不报告虚构的准确率；保留未能辨识位置，未补写回答或公式。完整时间覆盖不等于每个字都已确定。
照片电子版：原先纯看图转录保留；另提供结合录音核对的24条版本。
问题清单：28项学术问题、14项格式意见及结尾调整方向，按主题归并，附回听位置。
外发状态：此前飞书上传因未授权失败；本轮只下载模型，未向云端转写服务上传音频。
'''
(ROOT/'06_处理与校订说明.txt').write_text(method,'utf-8-sig')

audit=dict(segments=176,covered_seconds=q4[-1]['end'],continuous=True,reviewed_segments=len(overrides),marked_segments=len(uncertain),q4_segments=len(q4),q8_segments=len(q8),sensevoice_segments=len(sv),recheck_segments=len(read_rows('recheck_q8.jsonl')),characters=sum(len(x['text']) for x in records),source_title='多元扰动下多智能体协同鲁棒性分析')
(WORK/'verification.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),'utf-8')
files=['打开这里_答辩整理.html','01_完整录音转写_校订稿.txt','02_手写笔记_录音核对版.txt','03_录音问题与修改意见.txt','04_待核字句索引.txt','05_机器识别原稿对照_未经校订.txt','06_处理与校订说明.txt','手写笔记电子版.txt']
with zipfile.ZipFile(ROOT/'答辩录音整理_20261111.zip','w',zipfile.ZIP_DEFLATED) as z:
    for name in files:z.write(ROOT/name,name)
with zipfile.ZipFile(ROOT/'答辩录音整理_20261111.zip') as z:
    assert z.testzip() is None and len(z.namelist())==len(files)
print(json.dumps(audit,ensure_ascii=False,indent=2))
