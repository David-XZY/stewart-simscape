function x=dpContext(task)
o=task.scene.hood.obstacles; assert(numel(o)<=6,'DP:Obstacles','At most six OBBs.');
slots=zeros(13,6);
for j=1:numel(o), slots(:,j)=[o(j).center_S;o(j).size(:);o(j).R_S(:,1);o(j).R_S(:,2);1]; end
x=[task.q0;task.qGoal;task.T;double(strcmp(task.family,'simple'));double(strcmp(task.family,'hood'));slots(:)];
end
