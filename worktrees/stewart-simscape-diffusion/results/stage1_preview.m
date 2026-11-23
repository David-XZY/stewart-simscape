addpath('D:/NPU/Matlab_code/stewart-simscape-diffusion/diffusion'); c=setupDiffusion();
fig=figure('Visible','off','Color','w','Position',[30,30,1400,720]);
bits=[-1,-1,-1;1,-1,-1;-1,1,-1;1,1,-1;-1,-1,1;1,-1,1;-1,1,1;1,1,1].';
faces=[1,2,4,3;5,6,8,7;1,2,6,5;3,4,8,7;1,3,7,5;2,4,8,6];
families={'simple','hood'};
for k=1:2
 s=load(fullfile(c.output,'stage1',[families{k},'_01.mat'])); task=s.task; ax=subplot(1,2,k); hold(ax,'on');
 for o=task.scene.hood.obstacles
  vertices=o.center_S+o.R_S*(bits.*o.halfSize);
  patch('Vertices',vertices.','Faces',faces,'FaceColor',[.4,.5,.6],'FaceAlpha',.3,'EdgeColor',[.3,.4,.5]);
 end
 tr=dpReconstruct(s.result.bundle,linspace(0,task.T,201));
 plot3(tr.q(1,:),tr.q(2,:),tr.q(3,:),'Color',[.12,.45,.7],'LineWidth',2.5);
 for i=[1,101,201]
  q=tr.q(:,i); R=rpy2rotmZYX(q(4:6));
  [cy,cz,cx]=cylinder(task.scene.objectCylinder.radius,28); cx=(cx-.5)*task.scene.objectCylinder.length;
  world=q(1:3)+R*(task.scene.objectCylinder.center_P+[cx(:).';cy(:).';cz(:).']);
  surf(reshape(world(1,:),size(cx)),reshape(world(2,:),size(cx)),reshape(world(3,:),size(cx)), ...
      'FaceColor',[.9,.5,.2],'FaceAlpha',.25+.2*(i==201),'EdgeColor','none');
 end
 q=task.qGoal; R=rpy2rotmZYX(q(4:6));
 for j=1:6
  a=task.model.A(:,j); b=q(1:3)+R*task.model.B(:,task.model.legMap(j));
  plot3([a(1),b(1)],[a(2),b(2)],[a(3),b(3)],'Color',[.2,.3,.5],'LineWidth',1);
 end
 plot3(task.q0(1),task.q0(2),task.q0(3),'go','MarkerFaceColor','g'); plot3(q(1),q(2),q(3),'ro','MarkerFaceColor','r');
 title([families{k},': nominal IHSID acceptance'],'Interpreter','none'); xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
 axis equal; view(35,25); grid on;
end
sgtitle('Stage 1: full-horizon IHSID / 10 mm clearance / diffusion training pending');
print(fig,fullfile(c.output,'stage1_preview.png'),'-dpng','-r120'); close(fig);
