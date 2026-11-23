function r=pgTimeLowerBound(path,model,s,errorBound)
assert(all(diff(s)>0)&&s(1)==0&&s(end)==1,'BD:Grid','Ordered complete grid required.');
f=bdPathFunction(path); Q=full(f(s)); p=pgPhysics(model);
L=full(p.point(Q,zeros(size(Q)),zeros(size(Q))));
r=bdVariationBound(L,model.actuator.ldotMax,model.actuator.ldotMax,errorBound);
r.s=s; r.pathHash=dpHash(path); r.scope='Sampled variation lower bound; numeric error allowance is not machine certification.';
end
