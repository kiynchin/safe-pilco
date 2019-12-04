function d_state = dynamics_ddi(state,control)
%DYNAMICS_DDI Summary of this function goes here
%   Detailed explanation goes here
m = state.mass
x0 = state.x;
x0_dot = state.x_vel;
y0 = state.y;
y0_dot = state.y_vel;
F = control.force;
theta = control.angle;

x_dot = x0_dot;
y_dot = y0_dot;

x_dd = F*cos(theta)/m;
y_dd = F*cos(theta)/m;


d_state = [x_dot;
           x_dd;
           y_dot;
           y_dd];

end

