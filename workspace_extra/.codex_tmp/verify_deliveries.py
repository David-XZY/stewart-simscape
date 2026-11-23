from pathlib import Path
import hashlib,json,subprocess,sys
mode=sys.argv[1]
root=Path(r'D:\NPU\Matlab_code\stewart-simscape-tracking-mpc-v6-50hz')
if mode=='full':
 name='stewart_tracking_mpc_clean_50hz_20260921_180911';stage=root/'results/delivery'/name;archive=root.parent/(name+'.7z')
else:
 state=json.loads(Path(__file__).with_name('review_package.json').read_text());name=state['name'];stage=Path(state['stage']);archive=Path(state['archive']);assert archive.stat().st_size<=50_000_000
def digest(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
seven=r'C:\Program Files\7-Zip\7z.exe'
subprocess.run([seven,'t',str(archive)],check=True)
verifyParent=root/'results/archive_verification'/name;verifyParent.mkdir(parents=True,exist_ok=False)
subprocess.run([seven,'x',str(archive),'-o'+str(verifyParent),'-y'],check=True)
extracted=verifyParent/name
manifest=json.loads((extracted/'PACKAGE_MANIFEST.json').read_text())
def verify():
 for e in manifest:
  p=extracted/e['path'];assert p.stat().st_size==e['bytes'] and digest(p)==e['sha256'],e['path']
verify()
for e in json.loads((extracted/'SOURCE_COPY_MANIFEST.json').read_text()):assert digest(root/e['path'])==e['sha256'],e['path']
converted=json.loads((extracted/'PACKAGE_MAT_SERIALIZATION.json').read_text())
count=29 if mode=='full' else 0
assert len(converted)==count and all(e['allVariablesExactlyEqual'] for e in converted)
for e in converted:
 assert digest(root/e['path'])==e['sourceSHA256'];assert digest(extracted/e['path'])==e['packagedSHA256']
campaign=extracted/'results/tracking_mpc/clean_research_20260921'
for e in json.loads((campaign/'campaign_source.json').read_text())['files']:assert digest(extracted/e['file'])==e['sha256'],e['file']
assert len(list(campaign.glob('*/rawRun.mat')))==count and len(list(campaign.glob('*/actual_constraints.csv')))==29
print('EXTRACTION_HASHES_PASS',len(manifest),flush=True)
log=None
if mode=='test_each':
 log=verifyParent/'extracted_matlab_test.log'
 cmd="restoredefaultpath;addpath('controller/tests');test_112_tracking_research('package_verification');s=jsondecode(fileread('results/tracking_mpc/clean_research_20260921/selected_final_overrides.json'));c=makeTracking50HzConfig(s);assert(c.Np==30 && c.rateWeight==.1 && c.disturbanceMode==\"off\");assert(startsWith(which('run_ch4_tracking_mpc_final'),pwd));fprintf('EXTRACTED_FINAL_PROFILE_PASS\\n');"
 subprocess.run([r'D:\MATLAB\R2025b\bin\matlab.exe','-batch',cmd,'-logfile',str(log)],cwd=extracted,check=True)
 assert 'RESEARCH_TEST_PASS' in log.read_text(encoding='utf8',errors='replace') and 'EXTRACTED_FINAL_PROFILE_PASS' in log.read_text(encoding='utf8',errors='replace')
 verify()
receipt=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=digest(archive),includedFiles=len(manifest)+1,fullTimeSeriesCases=29,includedRawRuns=count,allMatVariablesExactlyEqual=True,sourceFilesUnchanged=True,archiveIntegrityPassed=True,allExtractionHashesPassed=True,extractedMatlabTestsPassed=(mode=='test_each'),verifiedExtraction=str(extracted),matlabLog=str(log) if log else None)
if mode=='review':
 full=json.loads((root.parent/'stewart_tracking_mpc_clean_50hz_20260921_180911.verification.json').read_text())
 assert full['extractedMatlabTestsPassed']
 fm=json.loads((Path(full['verifiedExtraction'])/'PACKAGE_MANIFEST.json').read_text());fm={e['path']:e['sha256'] for e in fm}
 code=[e for e in manifest if e['path'].startswith(('controller/','lib/','mat/','matlab/','opt_minimal/','src/'))]
 assert all(e['sha256']==fm[e['path']] for e in code)
 receipt.update(identicalCodeToTestedFullPackage=True,matchedSourceAndDependencyFiles=len(code),sharedMatlabTestLog=full['matlabLog'])
archive.with_suffix('.7z.sha256').write_text(receipt['sha256']+'  '+archive.name+'\n',encoding='utf8')
archive.with_suffix('.verification.json').write_text(json.dumps(receipt,indent=2),encoding='utf8')
print(json.dumps(receipt),flush=True)
