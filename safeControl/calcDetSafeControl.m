% Calculates safe control based on safety index and nominal controller.
% Assumes full model knowledge.
% Input: State q, nominal control u_nom, obstacles obst
% Output: Safety index phi, safe control u_safe

function [u_safe, phi] = calcDetSafeControl(q, u_nom, obst, box, lb, ub, K)
optArgIdx = 5;
% if we don't specify lower/upper bounds, set to inf
if(nargin < optArgIdx)
    lb = -inf*ones(size(u_nom));
end
if(nargin < optArgIdx + 1)
    ub = inf*ones(size(u_nom));
end
% if we don't specify gain, set to default
if(nargin < optArgIdx + 2)
    K = [-10, -3];
end
dist = 1;

[H, f, A, b, phi] = makeQPConstraints(q, u_nom, obst, dist, box, K);

u_safe = quadprog(H, f, A, b, [], [], lb, ub);

if(1)
    v = -10:0.01:10; [x1 x2] = meshgrid(v);
    cond = double(abs(x1) + abs(x2) >= 0);
    for ii = 1:size(A, 1)
        condN = double(A(ii,1)*x1 + A(ii,2)*x2 <= b(ii));
        condN(condN == 0) = NaN;
        cond = cond.*condN;
    end
    imagesc(flip(cond));
end
end

% Builds matrices describing quadratic program, along with safety index for
% each obstacle.
function [H, f, A, b, phi] = makeQPConstraints(q, u_nom, obst, dist, box, K)
H = eye(length(u_nom)); f = -u_nom;

A = []; b = []; phi = [];

for ii = 1:size(obst, 1)
    [indx, Am, bm] = calcSafetyIndexEllipse(q, obst(ii,:), dist, K);
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
    r1 = q(1) - x1_c; r2 = q(2) - x2_c;
    
    phi = (r1/a1)^2 + (r2/a2)^2 - dist;
    phi_d = 2*r1*q(3)/a1^2 + 2*r2*q(4)/a2^2;
    
    A = -[2*r1/a1^2, 2*r2/a2^2]; b = -K*[phi, phi_d]';
end

function [phi, A, b] = calcSafetyIndexBox(q, box, dist, K)
    left = box(1); right = box(2); lower = box(3); upper = box(4);
    
    phi = [q(1) - left - dist; right - q(1) - dist; q(2) - lower - dist; upper - q(3) - dist];
    A = [-1, 0; 1, 0; 0, -1; 0, 1]; b = -K*[phi(1), q(3); phi(2), -q(3); phi(3), q(4); phi(4), -q(4)]';
end