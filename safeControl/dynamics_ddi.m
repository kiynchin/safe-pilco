function d_q = dynamics_ddi(control)
%DYNAMICS_DDI Summary of this function goes here
%   Detailed explanation goes here
global q
m = q.mass;
x0 = q.x;
x0_dot = q.x_vel;
y0 = q.y;
y0_dot = q.y_vel;
F = control.force;
theta = control.angle;

x_dot = x0_dot;
y_dot = y0_dot;

x_dd = F*cos(theta)/m;
y_dd = F*sin(theta)/m;


d_q = [x_dot;
           y_dot;
           x_dd;
           y_dd];

end

