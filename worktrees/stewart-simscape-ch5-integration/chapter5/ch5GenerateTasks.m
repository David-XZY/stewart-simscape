function tasks = ch5GenerateTasks(split,c,physical,scene,collision)
switch split
    case 'train',count=c.train_tasks;seed=c.train_task_seed;
    case 'validation',count=c.validation_tasks;seed=c.validation_task_seed;
    case 'development',count=c.development_tasks;seed=c.development_task_seed;
    case 'test',count=12*c.tasks_per_cell;seed=c.test_task_seed;
    otherwise,error('ch5:Split','Unknown split.');
end
stream=RandStream('mt19937ar','Seed',seed);tasks=cell(count,1);
for index=1:count
    if strcmp(split,'test')
        family=1+floor((index-1)/(3*c.tasks_per_cell));level=1+mod(floor((index-1)/c.tasks_per_cell),3);
    else,family=1+mod(index-1,4);level=1+mod(floor((index-1)/4),3);end
    accepted=false;
    for attempt=1:10000
        q0=physical.qHome+[.06*(rand(stream,3,1)-.5)+[0;0;.16];deg2rad(8)*(rand(stream,3,1)-.5)];
        direction=randn(stream,3,1);direction=direction/norm(direction);
        if family==1,dist=c.displacements(level);rot=c.rotations_degrees(level);else,dist=.12+.08*rand(stream);rot=5+5*rand(stream);end
        goal=q0;goal(1:3)=goal(1:3)+dist*direction;goal(4:6)=rotate(q0(4:6),rot,stream);
        if ~legal(q0,physical) || ~legal(goal,physical),continue;end
        o=zeros(22,c.max_obstacles);o(7:15,:)=repmat(reshape(eye(3),9,1),1,c.max_obstacles);
        visible=zeros(1,c.max_obstacles);changeTimes=[];goals=goal;motion=zeros(3,c.max_obstacles);turn=zeros(1,c.max_obstacles);
        subtype=0;kinds=repmat("linear",1,c.max_obstacles);frequency=zeros(1,c.max_obstacles);
        amplitude=frequency;axes=zeros(3,2,c.max_obstacles);
        midpoint=(q0(1:3)+goal(1:3))/2;
        if family==2
            % Two parallel walls: clearance includes both the platform and payload.
            n=cross(direction,[0;0;1]);if norm(n)<.1,n=cross(direction,[0;1;0]);end;n=n/norm(n);
            b=[direction,n,cross(direction,n)];qmid=(q0+goal)/2;
            support=max([extent(q0,n,physical,scene),extent(goal,n,physical,scene),extent(qmid,n,physical,scene)]);
            width=support+c.safe_distance+c.passage_extra_width(level)/2;
            for j=1:2
                o(:,j)=[midpoint+(-1)^j*n*(width+.025);.8;.025;.4;reshape(b,9,1);zeros(6,1);1];
            end
            for j=3:2+c.clutter_counts(level)
                offset=randn(stream,3,1);offset=offset/norm(offset);
                o(:,j)=[midpoint+offset*(.7+.5*rand(stream));.03+.07*rand(stream,3,1);reshape(ch5Rotation(.8*randn(stream,3,1)),9,1);zeros(6,1);1];
            end
        elseif family==3
            if strcmp(split,'test'),subtype=1+mod(index-1,4);
            else,subtype=1+mod(floor((index-1)/4),4);end
            changeTimes=4;
            % Leave seven seconds after the final event: the baseline's
            % six-second plan and the 0.5-second hold fit within the deadline.
            if subtype==4,changeTimes=c.target_change_times(:)';end
            for j=1:numel(changeTimes)
                next=goals(:,end);
                if subtype~=2,next(1:3)=next(1:3)+c.switch_displacements(level)*direction*(-1)^(j-1);end
                if subtype~=1,next(4:6)=rotate(next(4:6),c.switch_rotations_degrees(level),stream);end
                goals(:,end+1)=next;
            end
        elseif family==4
            speed=c.obstacle_speeds(level);subtype=1+mod(index-1,3);
            R=ch5Rotation(goal(4:6));side=sign(R(:,2)'*(goal(1:3)-q0(1:3)));if side==0,side=1;end
            B=R*diag([side,side,1]);normal=B(:,2);axial=B(:,3);plate=goal(1:3)+R*[0;0;-.01];
            half=[.02;.02;.002];closestGap=.01;
            center=plate+normal*(physical.rB+half(2)+closestGap);
            if subtype==1
                % Cross the thin platform edge, not an entire metre-wide
                % platform at 5 mm/s. The goal is clear again before 15 s.
                entry=1.7;center=center-axial*(.01+half(3)+c.safe_distance+c.sample_hold_margin+speed*entry);
                motion(:,1)=speed*axial;visible(1)=entry-c.visibility_seconds(level);
            else
                half(3)=.01;center=plate+normal*(physical.rB+half(2)+c.safe_distance+.008);
                kinds(1)="approach";if subtype==3,kinds(1)="orbit";end
                frequency(1)=2*pi/c.episode_seconds;amplitude(1)=speed/frequency(1);axes(:,:,1)=[normal,axial];
            end
            o(:,1)=[center;half;B(:);zeros(6,1);1];
            if subtype>1
                probe=struct('goal_times',0,'goals',goal,'obstacles',o,'visible_times',visible, ...
                    'motion_velocity',motion,'turn_rate',turn,'motion_kind',kinds,'motion_frequency',frequency, ...
                    'motion_amplitude',amplitude,'motion_axes',axes);
                first=nan;
                for tt=0:.02:c.episode_seconds
                    [~,~,truth]=ch5Observe(probe,tt);
                    if min(full(collision.value(goal,truth)))<c.safe_distance+c.sample_hold_margin,first=tt;break;end
                end
                if ~isfinite(first),continue;end
                visible(1)=max(0,first-c.visibility_seconds(level));
            end
        end
        ok=true;for j=1:size(goals,2),ok=ok&&legal(goals(:,j),physical);end
        if ~ok,continue;end
        gap0=full(collision.value(q0,o));gapg=full(collision.value(goals(:,1),o));
        if min([gap0;gapg])<c.safe_distance,continue;end
        finalProbe=struct('goal_times',[0,changeTimes],'goals',goals,'obstacles',o,'visible_times',visible, ...
            'motion_velocity',motion,'turn_rate',turn,'motion_kind',kinds,'motion_frequency',frequency, ...
            'motion_amplitude',amplitude,'motion_axes',axes);
        [~,~,lastGeometry]=ch5Observe(finalProbe,c.episode_seconds);
        if min(full(collision.value(goals(:,end),lastGeometry)))<c.safe_distance+c.sample_hold_margin,continue;end
        accepted=true;break;
    end
    assert(accepted,'ch5:TaskGeneration','Could not draw a physically admissible task; no controller was used for screening.');
    tasks{index}=struct('id',sprintf('%s_%03d',split,index),'split',split,'seed',seed,'index',index, ...
        'family',family,'difficulty',level,'q0',q0,'qd0',zeros(6,1),'goals',goals, ...
        'goal_times',[0,changeTimes],'obstacles',o,'visible_times',visible, ...
        'motion_velocity',motion,'turn_rate',turn,'motion_kind',kinds,'motion_frequency',frequency, ...
        'motion_amplitude',amplitude,'motion_axes',axes,'subtype',subtype,'generation_attempts',attempt, ...
        'acceptance','Physical configuration and endpoint geometry only; no method performance screening');
end
end
function yes=legal(q,m)
if abs(q(5))>deg2rad(65),yes=false;return;end
k=sgpIK(q,m);j=sgpJacobian(q,m);
yes=all(k.L>=m.lmin+.01 & k.L<=m.lmax-.01)&&j.sigmaMin>=m.sigmaMinSafe+.01;
sigmaCertificate=sqrt(1/trace((j.Jbar*j.Jbar')\eye(6)));
gravity=-m.dynamics.totalMass*m.g;center=ch5Rotation(q(4:6))*m.dynamics.comP;
equilibrium=j.Jv'\[gravity;cross(center,gravity)];
yes=yes&&sigmaCertificate>=m.sigmaMinSafe+.002&&all(equilibrium>=m.actuator.forceMin&equilibrium<=m.actuator.forceMax);
end
function rpy=rotate(rpy,degrees,stream)
a=randn(stream,3,1);a=a/norm(a);K=[0,-a(3),a(2);a(3),0,-a(1);-a(2),a(1),0];t=deg2rad(degrees);
rpy=ch5Rotation((eye(3)+sin(t)*K+(1-cos(t))*K*K)*ch5Rotation(rpy));
end
function e=extent(q,n,m,s)
R=ch5Rotation(q(4:6));a=R*s.objectCylinder.axis_P;delta=R*s.objectCylinder.center_P;
e=max(m.rB+.01,abs(n'*delta)+abs(n'*a)*s.objectCylinder.length/2+s.objectCylinder.radius*sqrt(max(0,1-(n'*a)^2)));
end
