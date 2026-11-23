"""Produce analysis-only four-method derivatives; leave frozen numerical files untouched."""
from pathlib import Path
HERE=Path(__file__).resolve().parent

s=(HERE/'summarize.py').read_text(encoding='utf-8')
s=s.replace('from prepare import OUT,read,write,sha,METHODS,BUDGETS','from prepare import OUT,read,write,sha,METHODS,BUDGETS\nMETHODS=[m for m in METHODS if m!="ihsid"]')
s=s.replace("OUT/'protocol.json'","OUT/'protocol_four.json'").replace("OUT/'analysis", "OUT/'analysis_four")
s=s.replace("'独立IHSID成功',",'').replace('//5','//4')
(HERE/'report_summarize4.py').write_text(s,encoding='utf-8')

s=(HERE/'report_figures.py').read_text(encoding='utf-8')
s=s.replace('from prepare import OUT,read,write,sha,METHODS,BUDGETS','from prepare import OUT,read,write,sha,METHODS,BUDGETS\nMETHODS=[m for m in METHODS if m!="ihsid"]')
s=s.replace(",'IHSID'",'').replace(",'#171717'",'')
s=s.replace("OUT/'analysis", "OUT/'analysis_four").replace("OUT/'protocol.json'","OUT/'protocol_four.json'")
s=s.replace('len(rates)==25','len(rates)==20').replace('range(5)','range(4)').replace('np.zeros(5)','np.zeros(4)').replace('ncol=5','ncol=4').replace('//5','//4')
s=s.replace('(i-2)*.16','(i-1.5)*.19').replace("ax.bar(x,y,.145", "ax.bar(x,y,.175")
s=s.replace("'精修','NLP','其他'","'精修','其他'").replace(",'IHSID建模与求解'",'').replace(",'#292929'",'')
s=s.replace('assert np.all(matrix[:4,4]==0)',"assert all(r['stageMeanSeconds']['NLP']==0 for r in rs)")
s=s.replace("'独立IHSID成功',",'').replace('五方法','四方法')
(HERE/'report_figures4.py').write_text(s,encoding='utf-8')

s=(HERE/'report_collect.py').read_text(encoding='utf-8')
s=s.replace("state['stage']=='completed'","state['stage']=='four_completed'").replace('10950','8760')
s=s.replace("['formal','analysis','verification','freeze.json','preflight.json','status.json']","['formal','analysis_four','verification','freeze.json','preflight.json','status.json','protocol_four.json','amendment_no_ihsid.json','scope_change_stop.json']")
s=s.replace("OUT/'protocol.json'","OUT/'protocol_four.json'").replace("OUT/'analysis", "OUT/'analysis_four")
s=s.replace("HERE/'report_figures.py'","HERE/'report_figures4.py'").replace('五方法','四方法')
s=s.replace("[OUT/'protocol_four.json',OUT/'freeze.json'","[OUT/'protocol_four.json',OUT/'amendment_no_ihsid.json',OUT/'freeze.json'")
(HERE/'report_collect4.py').write_text(s,encoding='utf-8')
print('Created scope-specific summary, figures and collector; original numerical sources unchanged.')
