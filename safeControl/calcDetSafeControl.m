% Calculates safe control based on safety index and nominal controller.
% Assumes full model knowledge.
% Input: State q, nominal control u_nom, environment env
%      q -- q.x, q.y, q.x_vel, q.y_vel, q.mass, q.radius
%      control -- control.force, control.angle
%      env -- env.left_wall, env_right_wall, env.top_wall, env.bottom_wall
%             env.width, env.height, env.obstacles{}
%      env.obstacles has .x, .y, .a, .b
% Output: Safety index phi, safe control u_safe

function [controlSafe, phi] = calcDetSafeControl(q_all, control, env, lb, ub, K)
q = [q_all.x; q_all.y; q_all.x_vel; q_all.y_vel]; mass = q_all.mass; dist = q_all.radius;
u_nom = [control.force*cos(control.angle); control.force*sin(control.angle)]/mass;
obst = env.obstacles; box = [env.left_wall, env.right_wall, env.bottom_wall, env.top_wall];

optArgIdx = 4;
% if we don't specify lower/upper bounds, set to inf
if(nargin < optArgIdx)
    lb = -inf*ones(size(u_nom));
else
    lb = lb*ones(size(u_nom));
end
if(nargin < optArgIdx + 1)
    ub = inf*ones(size(u_nom));
else
    ub = ub*ones(size(u_nom));
end
% if we don't specify gain, set to default
if(nargin < optArgIdx + 2)
    K = [-10, -3];
end

[H, f, A, b, phi] = makeQPConstraints(q, u_nom, obst, dist, box, K);

opts = optimoptions(@quadprog, 'Display', 'off');
u_safe = quadprog(H, f, A, b, [], [], lb, ub, [], opts);
controlSafe.force = sqrt(u_safe(2)^2 + u_safe(1)^2)*mass;
controlSafe.angle = atan2(u_safe(2), u_safe(1));

if(1)
    v = -100:0.1:100; [x1 x2] = meshgrid(v);
    cond = double(abs(x1) + abs(x2) >= 0);
    for ii = 1:size(A, 1)
        condN = double(A(ii,1)*x1 + A(ii,2)*x2 <= b(ii));
        condN(condN == 0) = NaN;
        cond = cond.*condN;
    end
    figure(256);
    imagesc(flip(cond));
end
end

% Builds matrices describing quadratic program, along with safety index for
% each obstacle.
function [H, f, A, b, phi] = makeQPConstraints(q, u_nom, obst, dist, box, K)
H = eye(length(u_nom)); f = -u_nom;

A = []; b = []; phi = [];

for ii = 1:size(obst, 1)
    obst_params = [obst{ii}.x, obst{ii}.y, obst{ii}.a, obst{ii}.b];
    [indx, Am, bm] = calcSafetyIndexEllipse(q, obst_params, dist, K);
    phi = [phi; indx];
    A = [A; Am];
    b = [b; bm];
end
[indx, Am, bm] = calcSafetyIndexBox(q, box, dist, K);
phi = [phi; indx]; A = [A; Am]; b = [b; bm'];

end

% Calculates safety index and constraints for a single ellipsoidal obstacle.
% Input: State q, ellipsoidal obstacle with [x1_c, x2_c, a1, a2]
% Output: Safety index phi; A matrix of constraint; b matrix of constraint
function [phi, A, b] = calcSafetyIndexEllipse(q, obst, dist, K)
    x1_c = obst(1); x2_c = obst(2); a1 = obst(3); a2 = obst(4);
    r1 = q(1) - x1_c; r2 = q(2) - x2_c; dist = dist + 1;
    
    phi = (r1/a1)^2 + (r2/a2)^2 - dist;
    phi_d = 2*r1*q(3)/a1^2 + 2*r2*q(4)/a2^2;
    
    A = -[2*r1/a1^2, 2*r2/a2^2]; b = -K*[phi, phi_d]';
end

function [phi, A, b] = calcSafetyIndexBox(q, box, dist, K)
    left = box(1); right = box(2); lower = box(3); upper = box(4);
    
    phi = [q(1) - left - dist; right - q(1) - dist; q(2) - lower - dist; upper - q(2) - dist];
    A = [-1, 0; 1, 0; 0, -1; 0, 1]; b = -K*[phi(1), q(3); phi(2), -q(3); phi(3), q(4); phi(4), -q(4)]';
end