function [bundle,report]=restoreAbfdFrozenTaskLabels(bundle,savedTask,frozenTask)
% Only two human-readable labels may differ after old Windows JSON decoding.
% Every numeric value, shape, identifier and other field must remain EXACT.
assert(isequaln(bundle.model,savedTask.model) && isequaln(bundle.scene,savedTask.scene), ...
    'ABFD:HoldoutReferenceContext','Reference and its saved generation task differ.');
originalHash=abfdHash(savedTask); corrected=savedTask; changed={};
for field={'model','scene'}
    name=field{1};
    if ~isequaln(corrected.(name).name,frozenTask.(name).name)
        changed{end+1}=[name,'.name']; %#ok<AGROW>
        corrected.(name).name=frozenTask.(name).name;
    end
end
assert(isequaln(corrected,frozenTask),'ABFD:HoldoutNumericContext', ...
    'Frozen task differs beyond the two display labels; do not repair numeric values or other fields.');
bundle.model=frozenTask.model; bundle.scene=frozenTask.scene;
report=struct('originalTaskHash',originalHash,'canonicalTaskHash',abfdHash(frozenTask), ...
    'restoredFields',{changed},'numericAndOtherFieldsExactlyEqual',true, ...
    'scope','Only display labels restored from the byte-hashed pre-training freeze; original downloaded MAT preserved; reference must be independently rebuilt and accepted');
end
