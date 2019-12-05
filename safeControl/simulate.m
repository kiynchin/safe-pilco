close all;
clear all;
%%
global q
global goal
q.x = 10;
q.y = 10;
q.x_vel = 0;
q.y_vel = 0;
q.mass = 1;
q.radius = 2;
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

kp = 1;
kd = 1000;
last_err = [goal.x - q.x; goal.y - q.y];
while stop == false
    err = [goal.x - q.x; goal.y - q.y];
    d_err = err-last_err;
    proportional = kp*err
    derivative = kd*d_err
    force_vec = proportional+derivative;
    control.force = norm(force_vec); 
    control.angle = atan2(goal.y-q.y,goal.x-q.x);
    dq = dynamics_ddi(control);
    update_state(dq,dt);
    robot.Position = [q.x - q.radius,q.y-q.radius, q.radius,q.radius];
    drawnow;
    if q.x>env.right_wall || q.y > env.top_wall || q.x < env.left_wall || q.y < env.bottom_wall 
        stop = true;
    end
    pause(dt)
    %disp([control.force,control.angle])
    last_err = err;
end
