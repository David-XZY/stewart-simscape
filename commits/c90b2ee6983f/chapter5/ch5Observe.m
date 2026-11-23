function [goal,observed,truth,event] = ch5Observe(task,time)
% This function belongs to the simulator. Only goal and observed reach a controller.
event=find(task.goal_times<=time+1e-10,1,'last');goal=task.goals(:,event);
truth=task.obstacles;
for j=find(truth(22,:)>.5)
    initial=task.motion_velocity(:,j);omega=task.turn_rate(j);
    if isfield(task,'motion_kind')&&ismember(string(task.motion_kind(j)),["approach","orbit"])
        frequency=task.motion_frequency(j);amplitude=task.motion_amplitude(j);axes=task.motion_axes(:,:,j);angle=frequency*time;
        translation=amplitude*(cos(angle)-1)*axes(:,1);velocity=-amplitude*frequency*sin(angle)*axes(:,1);
        if string(task.motion_kind(j))=="orbit"
            translation=translation+amplitude*sin(angle)*axes(:,2);
            velocity=velocity+amplitude*frequency*cos(angle)*axes(:,2);
        end
    elseif abs(omega)<1e-12,translation=time*initial;velocity=initial;
    else
        angle=omega*time;K=[0,-1,0;1,0,0;0,0,0];
        translation=(sin(angle)/omega*eye(3)+(1-cos(angle))/omega*K)*initial;
        translation(3)=time*initial(3);
        velocity=[cos(angle),-sin(angle),0;sin(angle),cos(angle),0;0,0,1]*initial;
    end
    truth(1:3,j)=truth(1:3,j)+translation;truth(16:18,j)=velocity;
end
observed=truth;hidden=(time<task.visible_times)|(truth(22,:)<=.5);
% Mask the entire unknown object, including its geometry and velocity. An
% inactive bit alone would still leak truth through the network condition.
observed(:,hidden)=0;observed(7:15,hidden)=repmat(reshape(eye(3),9,1),1,sum(hidden));
end
