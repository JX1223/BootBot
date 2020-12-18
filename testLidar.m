addpath("./gen");
constants;

s_walker = [2, 2, deg2rad(30), zeros(1, 11)]';
s_ball = [9, 2]';

[angles, ranges] = readLIDAR(s_walker, s_ball);

pHead = pHead_gen(s_walker);
q_torso = s_walker(3);


% Plot
figure;
% Plot hip location
scatter(s_walker(1), s_walker(2), 'filled', 'MarkerFaceColor', 'blue');
hold on;
% Plot head (sensor) location
scatter(pHead(1), pHead(2), 'filled', 'MarkerFaceColor', 'green');
% Plot torso link
line([s_walker(1); pHead(1)], [s_walker(2), pHead(2)], 'Color', 'k', 'LineWidth', 1)
% Draw ball
rectangle('Position',[s_ball(1)-ball_radius, s_ball(2)-ball_radius, 2*ball_radius, 2*ball_radius], 'Curvature', [1 1], 'LineWidth', 1);

% Estimate Position of Ball
[position, radius] = estimateBallPosition(angles, ranges);
% Transforms ball position from sensor frame to world frame
pos_world = ([cos(-q_torso) -sin(-q_torso); sin(-q_torso) cos(-q_torso)] * position) + pHead;

scatter(pos_world(1), pos_world(2), 'MarkerEdgeColor', 'red');
scatter(s_ball(1), s_ball(2), 'x', 'MarkerEdgeColor', 'black');


% Calculate rays
% Positive angle convention is CCW, but q_torso is a CW angle, so need to
% add negative of q_torso:
angles = angles + (-1*q_torso);
U = ranges' .* cos(angles');
V = ranges' .* sin(angles');
X = pHead(1) * ones(size(U));
Y = pHead(2) * ones(size(U));

% Draw rays
quiver(X, Y, U, V, 0, 'ShowArrowHead', 'off', 'Color', 'red');

axis equal;

hold off;



