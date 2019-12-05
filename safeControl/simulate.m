close all;
clear all;
%%
global q
global goal
q.x = 10;
q.y = 42;
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

kp = 4;
kd = 1;
while stop == false
    err = pdist([q.x,q.y; goal.x, goal.y])
    %robot_vel = norm([q.x_vel,q.y_vel]);
    control.force = kp*err ;%- kd*robot_vel;
    control.angle = atan2(goal.y-q.y,goal.x-q.x);
    dq = dynamics_ddi(control);
    update_state(dq,dt);
    robot.Position = [q.x - q.radius,q.y-q.radius, q.radius,q.radius];
    drawnow;
    if q.x>env.right_wall || q.y > env.top_wall || q.x < env.left_wall || q.y < env.bottom_wall 
        stop = true;
    end
    pause(dt)
    disp([control.force,control.angle])

end
