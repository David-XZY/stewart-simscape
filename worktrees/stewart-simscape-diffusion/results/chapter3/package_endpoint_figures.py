import hashlib,json,pathlib,shutil,subprocess,sys,zipfile
import numpy,matplotlib,fitz
P=pathlib.Path(__file__).resolve().parent;O=P/'payload_endpoint_pass_figures_v1_20260927';R=P/'tmp/endpoint_five_reproduction_check'
compared=[]
for f in (R/'data').glob('*.json'):
 assert json.loads(f.read_text(encoding='utf8'))==json.loads((O/'data'/f.name).read_text(encoding='utf8')),f.name
 compared.append(f.name)
for name in ['plot_endpoint_five_methods.py','deliver_endpoint_five_methods.py','render_endpoint_atlas.cjs','audit_endpoint_figures.py','verify_endpoint_package.py']:
 shutil.copy2(P/name,O/name)
text='''先打开 index.html；PDF为8页图册；figures有7张PNG与7张可编辑文字SVG。
data有7884条过滤后完整调用日志、JSON和CSV绘图数据、排除清单、配对值及审计结果。
该包只分析原版本五种方法；原始数值代码、模型和数据身份见analysis_config.json。
不启动训练、不启动规划器、不启动补测。14个间隙拒绝任务没有被称作已证明无解。

在本包目录执行以下命令，可以不依赖原始大包重画：
python plot_endpoint_five_methods.py --snapshot . --output ../five_methods_reproduced
python deliver_endpoint_five_methods.py --output ../five_methods_reproduced
node render_endpoint_atlas.cjs ../five_methods_reproduced

Python绘图需要numpy、matplotlib和Microsoft YaHei中文字体。
PDF导出需要Node.js、playwright和Edge；异机可设置BROWSER_PATH指定浏览器路径。
图册及网页无需这些依赖，直接打开即可。
python verify_endpoint_package.py 验证全部文件SHA256及调用数量。

audit_endpoint_figures.py用于从项目results/chapter3目录复核原始大包来源；它需要同目录的
Stewart_complete_report_20260926/data。交付包内重画不需要原始大包。
sourceSha256保留原始三个归档文件的哈希；MANIFEST.json记录本包文件哈希。
复现只涉及分析；没有重新运行任何规划调用。SVG/PDF元数据时间可变化，统计JSON须一致。

统计说明：任务等权；20个场景成簇、3个跨场景共享区组重抽2000次；按重抽样总数比计算。
共同成功子集248组调用来自83任务和15场景，不是248个独立任务。
未测保持空缺；IHSID已测但零成功保持0。补测说明见补测与复现.txt。
轨迹质量为原日志保存的统一5ms指标；安全图为保存的完整报告极值。
最终成功标志包含独立复查与被选中时的加密复查，图中极值不冒称重新加密计算结果。
仅有3个训练种子、20个场景，区间解释须保留样本量限制；各指标分别展示，不使用总分。
'''
(O/'阅读与复现.txt').write_text(text,encoding='utf8')
pdf=fitz.open(O/'五方法结果图册.pdf');assert len(pdf)==8
assert all(p.get_text().count(chr(65533))==0 for p in pdf)
qa={'pdfPages':8,'pdfReplacementGlyphs':0,'popplerAllPagesVisuallyReviewed':True,'htmlAllCalls':7884,'htmlMissingCellRows':0,'htmlMeasuredCellRows':438,'browserErrors':0,'standaloneReproductionComparedJsonFiles':compared,'numpy':numpy.__version__,'matplotlib':matplotlib.__version__,'python':sys.version}
(O/'data/delivery_qa.json').write_text(json.dumps(qa,ensure_ascii=False,indent=2),encoding='utf8')
files=[{'path':p.relative_to(O).as_posix(),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(O.rglob('*')) if p.is_file() and p.name!='MANIFEST.json']
(O/'MANIFEST.json').write_text(json.dumps({'files':files,'selfExcluded':True},ensure_ascii=False,indent=2),encoding='utf8')
subprocess.run([sys.executable,str(O/'verify_endpoint_package.py')],check=True)
archive=P/'五方法端点通过结果图包_20260927.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
 for p in sorted(O.rglob('*')):
  if p.is_file():z.write(p,arcname=O.name+'/'+p.relative_to(O).as_posix())
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
assert archive.stat().st_size<50_000_000
digest=hashlib.sha256(archive.read_bytes()).hexdigest()
archive.with_suffix('.zip.sha256').write_text(digest+'  '+archive.name+'\n',encoding='utf8')
print(json.dumps({'zipBytes':archive.stat().st_size,'MB':archive.stat().st_size/1e6,'files':len(files)+1,'sha256':digest,'pdfPages':len(pdf)}))
