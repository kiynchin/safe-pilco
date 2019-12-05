function d_state = dynamics_ddi(state,control)
%DYNAMICS_DDI Summary of this function goes here
%   Detailed explanation goes here
m = 1;

x0 = state(1);
x0_dot = state(2);
y0 = state(3);
y0_dot = state(4);
F = control(1);
theta = control(2);

x_dot = x0_dot;
y_dot = y0_dot;

x_dd = F*cos(theta)/m;
y_dd = F*cos(theta)/m;


d_state = [x_dot;
           x_dd;
           y_dot;
           y_dd];

end

