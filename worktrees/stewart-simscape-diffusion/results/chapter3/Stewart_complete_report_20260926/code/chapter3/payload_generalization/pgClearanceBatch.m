function g=pgClearanceBatch(Q,task)
g=zeros(numel(task.scene.hood.obstacles),size(Q,2));
for k=1:size(Q,2), g(:,k)=pgClearance(Q(:,k),task,false).'; end
end
