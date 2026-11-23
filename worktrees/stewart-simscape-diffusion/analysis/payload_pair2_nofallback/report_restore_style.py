"""Reuse the supplied booklet's plotting code; alter only inputs and data dimensions."""
from pathlib import Path
H=Path(__file__).resolve().parent;root=H.parents[1]
s=(root/'results/chapter3/redraw_selected_five_methods.py').read_text(encoding='utf-8')
start=s.index('P=pathlib.Path(');end=s.index("M=['ddpm_basic'",start)
s=s[:start]+"from prepare import OUT as CAMPAIGN\nP=CAMPAIGN\nSRC=CAMPAIGN/'analysis_carry/data'\nOUT=CAMPAIGN/'analysis_carry/figures';OUT.mkdir(parents=True,exist_ok=True)\n"+s[end:]
s=s.replace('[1,2,5,10]','[1,2,5,10,20]')
s=s.replace("r['status']=='measured'","r['status'] in ['measured','budget_coverage']")
s=s.replace('assert len(v)==248 and np.isfinite(v).all()','assert len(v)==len(matched)//5 and np.isfinite(v).all()')
s=s.replace("next(r['calls'] for r in outcomes if r['B']==10 and r['method']==m and r['outcome']==name)","next((r['calls'] for r in outcomes if r['B']==10 and r['method']==m and r['outcome']==name),0)")
# Keep all six plotting sections verbatim, including titles, stage labels, colors and sizes.
s=s[:s.index("meta={'scope'")]+'''
meta={'scope':'original booklet presentation; new measurements and explicitly recorded source reuse in separate data tables',
      'methodOrder':M,'methodLabels':N,'methodColors':C,'qualityMatchedCalls':len(matched)//5,
      'qualityTasks':len({r['taskId'] for r in matched}),'qualityScenes':len({r['baseSceneId'] for r in matched}),
      'dataSha256':used,'plotSource':'results/chapter3/redraw_selected_five_methods.py'}
(OUT.parent/'figure_provenance.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT.parent/'index.html').write_text('<meta charset="utf-8"><title>五方法结果图</title>'+''.join('<img style="width:100%;max-width:1400px" src="figures/'+p.name+'">' for p in sorted(OUT.glob('*.png'))),encoding='utf-8')
import sys,subprocess
if '--pdf' in sys.argv:
 marker='C:/Users/David_X/.cache/codex-runtimes/codex-primary-runtime/plugins/openai-primary-runtime/plugins/pdf/skills/pdf/container_tools/mark_artifact_operation_started.mjs'
 subprocess.run(['node',marker,'--operation-kind','create','--expected-output-count','1','--output-format','pdf'],check=True)
 import fitz
 doc=fitz.open()
 for p in sorted(OUT.glob('*.png')):
  page=doc.new_page(width=1190.55,height=841.89);page.insert_image(fitz.Rect(22,22,1168.55,819.89),filename=str(p),keep_proportion=True)
 doc.save(OUT.parent/'五方法预算成功沿用图册.pdf');doc.close()
'''
(H/'report_figures_carry.py').write_text(s,encoding='utf-8')
print('Original figure text, layout, colors and method order restored; budgets now include 20s.')
