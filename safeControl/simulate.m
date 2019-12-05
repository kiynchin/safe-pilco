close all;
clear all;
%%
global q
global goal
start = [10,10];
q.x = start(1);
q.y = start(2);
q.x_vel = 0;
q.y_vel = 0;
q.mass = 1;
q.radius = 1;
env = createEnvironment();

dt = 0.01;
goal.x = 69;
goal.y = 42;

figure

drawEnvironment();
hold on
scatter(goal.x,goal.y,'r');
hold off
robot = drawRobot();
daspect([1,1,1]);

stop=false;

% kp = 4;
% kd = 4;
err = [goal.x - q.x; goal.y - q.y];
last_err = err;
height = err(2);
width = err(1);
phase =1 ;
while stop == false
%     err = [goal.x - q.x; goal.y - q.y];
%     d_err = err-last_err;
% 
%     proportional = kp*err;
%     derivative = kd*(d_err/dt);
%     force_vec = proportional+derivative;
    if phase == 1
    if ( q.y-start(2) )< height/2
        force_vec = [0; 50];
    end
    
    if (q.y-start(2)) >= height/2-0.5 &&( q.y-start(2))<height
        force_vec = [0;-50];
    end
    end
    
    if ( (q.y-start(2))>= height)
        phase=2;
    end
    
    if phase == 2
    if (q.x - start(1)) <width/2
        force_vec = [50;0];
    end
    
    if  q.x - start(1) >width/2 && q.x - start(1) <width
        force_vec = [-50;0];
    end
    end
    
    
    
    control.force = min(norm(force_vec), 100); 
    control.angle = atan2(force_vec(2),force_vec(1));
    [safeControl, phi] = calcDetSafeControl(q, control, env);
    
    dq = dynamics_ddi(control);
    update_state(dq,dt);
    robot.Position = [q.x - q.radius,q.y-q.radius, q.radius,q.radius];
    drawnow;
    if q.x>env.right_wall || q.y > env.top_wall || q.x < env.left_wall || q.y < env.bottom_wall 
        stop = true;
    end
    pause(dt)
    disp([control.force,control.angle])
    disp([safeControl.force,safeControl.angle]);
    disp(phi);
%     last_err = err;
end
