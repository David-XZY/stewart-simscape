function renderDiffusionResults(runName)
c=setupDiffusion(); folder=fullfile(c.output,runName); root=fullfile(folder,'figures'); if ~isfolder(root), mkdir(root); end
files=dir(fullfile(folder,'benchmark','*','*','benchmark.mat')); chosen=struct('family',{},'file',{});
for k=1:numel(files)
    s=load(fullfile(files(k).folder,files(k).name));
    if strcmp(s.row.method,'guided_diffusion') && s.row.passed && ~any(strcmp({chosen.family},s.task.family))
        chosen(end+1)=struct('family',s.task.family,'file',fullfile(files(k).folder,files(k).name)); %#ok<AGROW>
    end
end
for k=1:numel(chosen)
    s=load(chosen(k).file); task=s.task; best=s.outcome.best; colors=[lines(7);.35,.35,.35];
    fig=figure('Visible','off','Color','w','Position',[50,50,1400,850]); ax=axes('Parent',fig); hold(ax,'on');
    drawObstacles(ax,task);
    routes=gobjects(1,9); labels=cell(1,9);
    for j=1:size(s.candidates,3)
        tr=dpSpline(s.candidates(:,:,j),linspace(0,task.T,301),task.T);
        routes(j)=plot3(ax,tr.q(1,:),tr.q(2,:),tr.q(3,:),'Color',colors(j,:),'LineWidth',1);
        labels{j}=sprintf('Candidate %d',j);
    end
    r=best.report; routes(9)=plot3(ax,r.q(1,:),r.q(2,:),r.q(3,:),'k','LineWidth',3); labels{9}='Accepted IHSID';
    plot3(ax,task.q0(1),task.q0(2),task.q0(3),'go','MarkerFaceColor','g');
    plot3(ax,task.qGoal(1),task.qGoal(2),task.qGoal(3),'ro','MarkerFaceColor','r');
    axis(ax,'equal'); grid(ax,'on'); view(ax,35,25); xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); zlabel(ax,'z (m)');
    title(ax,sprintf('%s: platform-center candidates and accepted IHSID trajectory',task.family),'Interpreter','none');
    legend(ax,routes,labels,'Location','eastoutside');
    print(fig,fullfile(root,[task.family,'_candidates.png']),'-dpng','-r150'); close(fig);
    before=dpSpline(best.sourceSeed,r.time,task.T);
    fig=figure('Visible','off','Color','w','Position',[50,50,1500,900]);
    labels={'x (m)','y (m)','z (m)','roll (deg)','pitch (deg)','yaw (deg)'};
    for j=1:6
        subplot(3,2,j); factor=1; if j>3, factor=180/pi; end
        plot(r.time,factor*before.q(j,:),'--','LineWidth',1.2); hold on;
        plot(r.time,factor*r.q(j,:),'LineWidth',1.4); grid on; ylabel(labels{j}); xlabel('Time (s)');
        legend('Selected diffusion candidate','Accepted IHSID refinement','Location','best');
    end
    print(fig,fullfile(root,[task.family,'_refinement.png']),'-dpng','-r150'); close(fig);
    fig=figure('Visible','off','Color','w','Position',[50,50,1500,1000]);
    subplot(3,2,1); plot(r.time,1000*r.clearance.'); hold on; yline(10,'r--'); title('Load/environment clearance'); ylabel('mm'); grid on;
    subplot(3,2,2); plot(r.time,r.L.'); hold on; yline(1.05,'r--'); yline(2,'r--'); title('Leg length (m)'); grid on;
    subplot(3,2,3); plot(r.time,r.Ld.'); hold on; yline(.6,'r--'); yline(-.6,'r--'); title('Leg speed (m/s)'); grid on;
    subplot(3,2,4); plot(r.time,r.Ldd.'); hold on; yline(1.2,'r--'); yline(-1.2,'r--'); title('Leg acceleration (m/s^2)'); grid on;
    subplot(3,2,5); plot(r.time,r.FRequired.'); hold on; yline(2000,'r--'); yline(-2000,'r--'); title('Unclipped required leg force (N)'); xlabel('Time (s)'); grid on;
    subplot(3,2,6); plot(r.time,r.q(4:6,:).'*180/pi); title('Platform RPY (deg)'); xlabel('Time (s)'); grid on;
    print(fig,fullfile(root,[task.family,'_constraints.png']),'-dpng','-r150'); close(fig);
    animate(task,best,fullfile(root,[task.family,'.mp4']));
end
dpWriteJson(fullfile(root,'render_manifest.json'),chosen);
end
function drawObstacles(ax,task)
faces=[1,2,4,3;5,6,8,7;1,2,6,5;3,4,8,7;1,3,7,5;2,4,8,6];
bits=[-1,-1,-1;1,-1,-1;-1,1,-1;1,1,-1;-1,-1,1;1,-1,1;-1,1,1;1,1,1].';
for o=task.scene.hood.obstacles
    vertices=o.center_S+o.R_S*(bits.*o.halfSize);
    patch(ax,'Vertices',vertices.','Faces',faces,'FaceColor',[.5,.6,.7],'FaceAlpha',.18,'EdgeColor',[.3,.4,.5]);
end
end
function animate(task,best,file)
fig=figure('Visible','off','Color','w','Position',[50,50,1000,760]); ax=axes('Parent',fig); hold(ax,'on'); drawObstacles(ax,task);
r=best.report; plot3(ax,r.q(1,:),r.q(2,:),r.q(3,:),'k--'); axis(ax,'equal'); grid(ax,'on'); view(ax,35,25);
xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); zlabel(ax,'z (m)'); title(ax,[task.family,' / accepted trajectory']);
all=r.q(1:3,:); xlim(ax,[min(all(1,:))-.8,max(all(1,:))+.8]); ylim(ax,[min(all(2,:))-.7,max(all(2,:))+.7]); zlim(ax,[.0,2.1]);
[cy,cz,cx]=cylinder(task.scene.objectCylinder.radius,28); cx=(cx-.5)*task.scene.objectCylinder.length;
obj=surf(ax,cx,cy,cz,'FaceColor',[.9,.5,.2],'EdgeColor','none','FaceAlpha',.8);
legs=gobjects(1,6); for j=1:6, legs(j)=plot3(ax,[0,0],[0,0],[0,0],'b-','LineWidth',2); end
[~,baseOrder]=sort(atan2(task.model.A(2,:),task.model.A(1,:)));
[~,platformOrder]=sort(atan2(task.model.B(2,:),task.model.B(1,:)));
patch(ax,'Vertices',task.model.A.','Faces',baseOrder,'FaceColor',[.6,.65,.7],'FaceAlpha',.3,'EdgeColor',[.3,.3,.3]);
platform=patch(ax,'Vertices',zeros(6,3),'Faces',platformOrder,'FaceColor',[.2,.6,.6],'FaceAlpha',.4,'EdgeColor',[.1,.3,.3]);
writer=VideoWriter(file,'MPEG-4'); writer.FrameRate=20; open(writer); cleanup=onCleanup(@()close(writer));
times=linspace(0,task.T,ceil(task.T*20)+1); tr=dpReconstruct(best.bundle,times);
for k=1:numel(times)
    R=rpy2rotmZYX(tr.q(4:6,k)); world=tr.q(1:3,k)+R*(task.scene.objectCylinder.center_P+[cx(:).';cy(:).';cz(:).']);
    set(obj,'XData',reshape(world(1,:),size(cx)),'YData',reshape(world(2,:),size(cx)),'ZData',reshape(world(3,:),size(cx)));
    set(platform,'Vertices',(tr.q(1:3,k)+R*task.model.B).');
    for j=1:6
        a=task.model.A(:,j); b=tr.q(1:3,k)+R*task.model.B(:,task.model.legMap(j));
        set(legs(j),'XData',[a(1),b(1)],'YData',[a(2),b(2)],'ZData',[a(3),b(3)]);
    end
    clearance=1000*min(dpClearance(tr.q(:,k),task,true));
    title(ax,sprintf('%s | t = %.2f s | clearance = %.2f mm',task.family,times(k),clearance),'Interpreter','none');
    drawnow; frame=print(fig,'-RGBImage','-r80'); writeVideo(writer,frame);
end
clear cleanup; close(fig);
end
