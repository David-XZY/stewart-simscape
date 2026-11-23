function [gaps,normals]=pgClearance(q,task,refine)
if nargin<3, refine=false; end
p=task.payload; R=rpy2rotmZYX(q(4:6))*p.mount.R; center=q(1:3)+rpy2rotmZYX(q(4:6))*p.mount.center(:);
obs=task.scene.hood.obstacles; gaps=zeros(1,numel(obs)); normals=zeros(3,numel(obs));
for j=1:numel(obs)
    o=obs(j); delta=o.center_S-center; dirs=[eye(3),R,o.R_S,delta];
    for k=1:3, for l=1:3, dirs=[dirs,cross(R(:,k),o.R_S(:,l))]; end, end %#ok<AGROW>
    dirs=[dirs,-dirs]; dirs=dirs(:,sqrt(sum(dirs.^2,1))>1e-12); dirs=dirs./sqrt(sum(dirs.^2,1));
    values=gap(dirs); [gaps(j),idx]=max(values); normals(:,j)=dirs(:,idx);
    if refine && gaps(j)<.01
        n=normals(:,j); angles=[atan2(n(2),n(1)),asin(max(-1,min(1,n(3))))];
        opt=optimset('Display','off','MaxIter',60,'MaxFunEvals',130);
        z=fminsearch(@(z)-gap(direction(z)),angles,opt); v=gap(direction(z));
        if v>gaps(j), gaps(j)=v; normals(:,j)=direction(z); end
    end
end
    function v=gap(n)
        local=R.'*n;
        if strcmp(p.shape,'cylinder'), support=p.dimensions(1)/2*abs(local(1,:))+p.dimensions(2)*sqrt(sum(local(2:3,:).^2,1));
        else, support=p.dimensions(:).'/2*abs(local); end
        v=delta.'*n-o.halfSize(:).'*abs(o.R_S.'*n)-support;
    end
end
function n=direction(z)
n=[cos(z(2))*cos(z(1));cos(z(2))*sin(z(1));sin(z(2))];
end
