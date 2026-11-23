from pathlib import Path
import hashlib,json,subprocess
state=json.loads(Path(__file__).with_name('package_clean_research.json').read_text())
root=Path(state['root']);stage=Path(state['stage']);archive=Path(state['archive'])
def digest(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
assert archive.stat().st_size<=50_000_000,archive.stat().st_size
seven=r'C:\Program Files\7-Zip\7z.exe'
subprocess.run([seven,'t',str(archive)],check=True)
verifyParent=root/'results/archive_verification'/state['name']
verifyParent.mkdir(parents=True,exist_ok=False)
subprocess.run([seven,'x',str(archive),'-o'+str(verifyParent),'-y'],check=True)
extracted=verifyParent/state['name']
manifest=json.loads((extracted/'PACKAGE_MANIFEST.json').read_text())
for e in manifest:
 p=extracted/e['path'];assert p.stat().st_size==e['bytes'] and digest(p)==e['sha256'],e['path']
for e in json.loads((extracted/'SOURCE_COPY_MANIFEST.json').read_text()):
 assert digest(root/e['path'])==e['sha256'],e['path']
converted=json.loads((extracted/'PACKAGE_MAT_SERIALIZATION.json').read_text())
assert len(converted)==29 and all(e['allVariablesExactlyEqual'] for e in converted)
for e in converted:
 assert digest(root/e['path'])==e['sourceSHA256']
 assert digest(extracted/e['path'])==e['packagedSHA256']
campaign=extracted/'results/tracking_mpc/clean_research_20260921'
for e in json.loads((campaign/'campaign_source.json').read_text())['files']:
 assert digest(extracted/e['file'])==e['sha256'],e['file']
assert len(list(campaign.glob('*/rawRun.mat')))==29
print('EXTRACTION_HASHES_PASS',len(manifest),flush=True)
log=verifyParent/'extracted_matlab_test.log'
cmd="restoredefaultpath;restore_campaign_csv;addpath('controller/tests');test_112_tracking_research('package_verification');s=jsondecode(fileread('results/tracking_mpc/clean_research_20260921/selected_final_overrides.json'));c=makeTracking50HzConfig(s);assert(c.Np==30 && c.rateWeight==.1 && c.disturbanceMode==\"off\");assert(startsWith(which('run_ch4_tracking_mpc_final'),pwd));fprintf('EXTRACTED_FINAL_PROFILE_PASS\\n');"
subprocess.run([r'D:\MATLAB\R2025b\bin\matlab.exe','-batch',cmd,'-logfile',str(log)],cwd=extracted,check=True)
assert 'RESEARCH_TEST_PASS' in log.read_text(encoding='utf8',errors='replace') and 'EXTRACTED_FINAL_PROFILE_PASS' in log.read_text(encoding='utf8',errors='replace')
for e in manifest:assert digest(extracted/e['path'])==e['sha256'],e['path']
assert 'RESTORED_CSV_HASHES_PASS files=87' in log.read_text(encoding='utf8',errors='replace')
receipt=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=digest(archive),includedFiles=len(manifest)+1,completeRawRuns=29,allMatVariablesExactlyEqual=True,sourceFilesUnchanged=True,archiveIntegrityPassed=True,allExtractionHashesPassed=True,regeneratedCsvHashesPassed=87,extractedMatlabTestsPassed=True,verifiedExtraction=str(extracted),matlabLog=str(log))
archive.with_suffix('.7z.sha256').write_text(receipt['sha256']+'  '+archive.name+'\n',encoding='utf8')
archive.with_suffix('.verification.json').write_text(json.dumps(receipt,indent=2),encoding='utf8')
print(json.dumps(receipt),flush=True)
