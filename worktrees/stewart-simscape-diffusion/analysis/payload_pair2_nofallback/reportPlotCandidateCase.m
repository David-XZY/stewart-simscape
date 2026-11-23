function reportPlotCandidateCase()
% Native MATLAB figure; only raw reference-point paths, endpoint frames and OBBs.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
out=fullfile(root,'results','chapter3','payload_pair2_nofallback_v1','candidate_case_v1');
d=load(fullfile(out,'candidate_plot_data.mat'));
set(groot,'defaultAxesFontName','Microsoft YaHei','defaultTextFontName','Microsoft YaHei');
fig=figure('Visible','off','Color','w','Position',[80,80,1420,920]);
ax=axes(fig,'Position',[.07,.12,.88,.78]); hold(ax,'on');
face=[1,2,3,4;5,6,7,8;1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8];
signs=[-1,-1,-1;1,-1,-1;1,1,-1;-1,1,-1;-1,-1,1;1,-1,1;1,1,1;-1,1,1].';
for k=1:numel(d.obstacles)
    o=d.obstacles(k); verts=o.center_S(:)+o.R_S*(signs.*o.halfSize(:));
    patch(ax,'Vertices',verts.','Faces',face,'FaceColor',[.53,.57,.64], ...
        'FaceAlpha',.25,'EdgeColor',[.35,.39,.46],'LineWidth',1.0,'HandleVisibility','off');
    text(ax,o.center_S(1),o.center_S(2),o.center_S(3)+o.halfSize(3)+.025, ...
        ['障碍物 ',char('A'+k-1)],'HorizontalAlignment','center','FontSize',13,'Color',[.22,.25,.3]);
end
for k=1:12
    q=d.candidateQ(:,:,k);
    h=plot3(ax,q(1,:),q(2,:),q(3,:),'Color',[.68,.77,.85],'LineWidth',1.1);
    if k==1, rejected=h; else, h.HandleVisibility='off'; end
end
q=d.candidateQ(:,:,14); untested=plot3(ax,q(1,:),q(2,:),q(3,:),'--','Color',[.8,.55,.27],'LineWidth',1.6);
q=d.acceptedQ; accepted=plot3(ax,q(1,:),q(2,:),q(3,:),'Color',[.0,.47,.40],'LineWidth',3.6);
scatter3(ax,d.q0(1),d.q0(2),d.q0(3),64,[.10,.28,.55],'o','filled','HandleVisibility','off');
scatter3(ax,d.qGoal(1),d.qGoal(2),d.qGoal(3),80,[.68,.20,.17],'d','filled','HandleVisibility','off');
frame(ax,d.q0,.075); frame(ax,d.qGoal,.075);
text(ax,d.q0(1)-.035,d.q0(2)-.06,d.q0(3)-.04, ...
    sprintf('起点\n(%.3f, %.3f, %.3f) m\nRPY = (0, 0, 0)°',d.q0(1:3)), ...
    'FontSize',11,'HorizontalAlignment','right','Color',[.1,.23,.38]);
text(ax,d.qGoal(1)+.02,d.qGoal(2)+.02,d.qGoal(3)+.21, ...
    sprintf('终点\n(%.3f, %.3f, %.3f) m\nRPY = (0, 0, 0)°',d.qGoal(1:3)), ...
    'FontSize',11,'HorizontalAlignment','left','Color',[.55,.17,.13]);
xlabel(ax,'X / m'); ylabel(ax,'Y / m'); zlabel(ax,'Z / m');
axis(ax,'equal'); grid(ax,'on'); box(ax,'on'); ax.FontSize=12; ax.GridAlpha=.13;
xlim(ax,[-.78,.78]); ylim(ax,[-.42,.43]); zlim(ax,[.94,1.46]);
view(ax,[35,27]); ax.Projection='orthographic';
title(ax,'扩散多候选轨迹与最终通过轨迹','FontSize',20,'FontWeight','normal');
legend(ax,[rejected,accepted,untested],{'候选1-12：未通过','候选13：直接通过','候选14：已生成、未检查'}, ...
    'Location','southoutside','Orientation','horizontal','Box','off','FontSize',12);
savefig(fig,fullfile(out,'扩散多候选三维轨迹.fig'));
exportgraphics(fig,fullfile(out,'扩散多候选三维轨迹.png'),'Resolution',300);
print(fig,fullfile(out,'扩散多候选三维轨迹.svg'),'-dsvg');
close(fig);
fprintf('MATLAB_PLOT_OK %s\n',out);
end
function frame(ax,q,len)
r=q(4); p=q(5); y=q(6);
Rx=[1,0,0;0,cos(r),-sin(r);0,sin(r),cos(r)];
Ry=[cos(p),0,sin(p);0,1,0;-sin(p),0,cos(p)];
Rz=[cos(y),-sin(y),0;sin(y),cos(y),0;0,0,1]; R=Rz*Ry*Rx;
colors=[.80,.22,.22;.23,.59,.26;.20,.37,.80];
for j=1:3
    v=len*R(:,j); endp=q(1:3)+v;
    quiver3(ax,q(1),q(2),q(3),v(1),v(2),v(3),0,'Color',colors(j,:), ...
        'LineWidth',1.6,'MaxHeadSize',.4,'HandleVisibility','off');
    text(ax,endp(1),endp(2),endp(3),[' ',char('x'+j-1)],'Color',colors(j,:),'FontSize',10);
end
end
