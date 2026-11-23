function [goal,observed,truth,event] = ch5Observe(task,time)
% This function belongs to the simulator. Only goal and observed reach a controller.
event=find(task.goal_times<=time+1e-10,1,'last');goal=task.goals(:,event);
truth=task.obstacles;
for j=find(truth(22,:)>.5)
    initial=task.motion_velocity(:,j);omega=task.turn_rate(j);
    if abs(omega)<1e-12,translation=time*initial;velocity=initial;
    else
        angle=omega*time;K=[0,-1,0;1,0,0;0,0,0];
        translation=(sin(angle)/omega*eye(3)+(1-cos(angle))/omega*K)*initial;
        translation(3)=time*initial(3);
        velocity=[cos(angle),-sin(angle),0;sin(angle),cos(angle),0;0,0,1]*initial;
    end
    truth(1:3,j)=truth(1:3,j)+translation;truth(16:18,j)=velocity;
end
observed=truth;observed(22,time<task.visible_times)=0;
end
