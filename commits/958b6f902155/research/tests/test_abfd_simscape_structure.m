function report=test_abfd_simscape_structure()
% Compile/wire only, no trajectory simulation and no trained assets.
root=setupAbfdResearch(); m=buildOptModelCustom(); s=buildCylinderBoxTransferScene(m);
p=buildDynamicsProvider(m); q=repmat(m.qHome,1,3); z=zeros(6,3); t=[0,.01,.02];
F=repmat(p.inverse(m.qHome,zeros(6,1),zeros(6,1)),1,3); kin=sgpIK(m.qHome,m);
refs=struct('q',q,'qd',z,'qdd',z,'Fleg',F,'L',repmat(kin.L,1,3),'t',t);
bundle=struct('schemaVersion',1,'publishPassed',true,'providerHash',p.hash, ...
    'model',m,'scene',s,'refs',refs,'status','test-fixture');
c=makeAbfdConfig(struct('controllerMode','geometric-D'));
file=fullfile(root,'matlab','stewart_platform_model.slx'); before=hashFile(file);
timer=tic; passed=false; message='';
try
    setup=prepareAbfdSimscape(bundle,p,c,makeHighFidelityPlantProfile('matched'));
    assert(~setup.interface.modelSaved);
    handles=get_param(setup.interface.controllerBlock,'PortHandles');
    assert(numel(handles.Inport)==3 && numel(handles.Outport)==6);
    line=get_param(handles.Outport(1),'Line'); dest=get_param(line,'DstBlockHandle');
    assert(any(strcmp(get_param(dest,'Name'),'Stewart Platform')));
    close_system(setup.modelName,0);
    load_system(file); assert(getSimulinkBlockHandle('stewart_platform_model/ABFD Controller')==-1);
    close_system('stewart_platform_model',0);
    assert(strcmp(before,hashFile(file))); passed=true;
catch e
    message=getReport(e,'extended','hyperlinks','off');
    if bdIsLoaded('stewart_platform_model'), close_system('stewart_platform_model',0); end
end
report=struct('passed',passed,'scope','in-memory installation, compilation and restoration; no simulation', ...
    'seconds',toc(timer),'message',message,'sourceUnchanged',strcmp(before,hashFile(file)));
writeAbfdJson(fullfile(root,'results','abfd','simscape_structure_check.json'),report);
fprintf('SIMSCAPE_STRUCTURE_PASS=%d\n',passed);
assert(passed,'ABFD:SimscapeStructureFailed','%s',message);
end
function out=hashFile(file)
fid=fopen(file,'rb'); clean=onCleanup(@()fclose(fid)); data=fread(fid,inf,'*uint8'); clear clean;
d=java.security.MessageDigest.getInstance('SHA-256'); d.update(data); out=char(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
