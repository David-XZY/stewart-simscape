c=setupDiffusion(); folder=fullfile(c.root,'results','pilot_v4_inspection');
s=load(fullfile(folder,'heldout_simple.mat'),'task'); task=s.task;
modelFile=fullfile(folder,'training','model.json'); checkNetworkExport(modelFile);
[candidates,generation]=sampleCandidates(task,modelFile,8,c.seed+task.seed,false,c);
diversity=dpDiversity(candidates,task); disp(diversity);
save(fullfile(folder,'actual_unguided_candidates.mat'),'task','candidates','generation','diversity');
f=figure('Visible','off','Color','w','Position',[50,50,1100,800]); hold on; colors=lines(8);
for k=1:8
    tr=dpSpline(candidates(:,:,k),linspace(0,task.T,201),task.T);
    plot3(tr.q(1,:),tr.q(2,:),tr.q(3,:),'Color',colors(k,:),'LineWidth',1.5,'DisplayName',sprintf('Candidate %d',k));
end
grid on; axis equal; view(35,25); xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('Location','eastoutside'); title({'Held-out simple task / actual trained DDPM','Unguided candidates before IHSID refinement'});
print(f,fullfile(folder,'actual_unguided_candidates.png'),'-dpng','-r120'); close(f);
