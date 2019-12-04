close all;
clear;
%%
state.x = 10;
state.y = 10;
state.x_vel = 0;
state.y_vel = 0;
state.mass = 10;
state.radius = 1;


figure
drawEnvironment;
drawRobot(state.x,state.y,state.radius)
drawObstacles;
daspect([1,1,1])

stop=false;
while stop == false
    
    stop=true;
end