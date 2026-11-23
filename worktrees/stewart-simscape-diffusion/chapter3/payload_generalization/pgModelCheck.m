function pgModelCheck(model,method,c)
signature=contains(method,'signature'); dim=99; schema=c.contextSchema;
if signature, dim=354; schema=c.signatureSchema; end
assert(strcmp(model.contextSchema,schema)&&numel(model.inputMean)==dim,'PG:Schema','Legacy or mismatched model conditions.');
assert(model.controlCount==c.controlCount && strcmp(model.outputRepresentation,'absolute_free_controls_column_major'),'PG:Representation','Output representation mismatch.');
expected=method; if any(strcmp(method,{'ddpm_guided','ddpm_end_guided'})), expected='ddpm_basic'; end
assert(strcmp(model.method,expected),'PG:ModelMethod','Incorrect model lineage.');
if startsWith(method,'ddpm')
    s=pgNoiseSchedule(100);
    for f={'beta','alphaBar','posteriorVariance','posteriorMeanClean','posteriorMeanNoisy'}
        assert(max(abs(model.noiseSchedule.(f{1})(:)-s.(f{1})(:)))<1e-12,'PG:Schedule','Training/sampling schedule mismatch.');
    end
end
end
