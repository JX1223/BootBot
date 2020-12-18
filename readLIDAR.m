% ME193b
% Jason Xu

function [angles, ranges] = readLIDAR(s_walker, s_ball, C)
    % Obtain constants
    lidar_range = C.lidar_range;
    noise_scale = C.noise_scale;
    ball_radius = C.ball_radius;
    lidar_FOV = C.lidar_FOV;
    lidar_rays = C.lidar_rays;
    lidar_noiseOn = C.lidar_noiseOn;
    % Extract coordinates
    pHead_e = pHead_gen(s_walker); % Position of head (sensor) in Earth frame
    pBall_e = [s_ball(1); s_ball(2)]; % Position of ball in Earth frame
    q_walker = q_gen(s_walker);
    q_torso = q_walker(3);
    % Normalize coordinated based on head and transform coordinates
    % into sensor (walker) frame.
    % Sensor is located at head, and points out in the x_b direction
    T_BE  = [cos(q_torso), -sin(q_torso); sin(q_torso), cos(q_torso)];
    pBall_b = T_BE * (pBall_e - pHead_e);
    
    % Generate points based on number of ray counts and FOV
    angles = linspace(-lidar_FOV/2, lidar_FOV/2, lidar_rays);
    p2 = [cos(angles); sin(angles)];
    p1 = zeros(size(p2)); % By our definition, pSensor_b located at [0; 0], so each ray originates at [0; 0]
    
    % Obtain intersection distance for each ray
    ranges = circleLine(p1, p2, pBall_b, ball_radius);
    
    % For rays that do not intersect, or are out of max range, set to max
    % distance
    ranges(ranges > lidar_range | ranges < 0) = lidar_range;
    
    % Add noise
    if lidar_noiseOn
        ranges_d = ranges(ranges < lidar_range);
        ranges(ranges < lidar_range) = ranges_d + ((noise_scale).*ranges_d)./lidar_range .* randn(size(ranges_d));
        %ranges(ranges < lidar_range) = ranges(ranges < lidar_range) + (noise_scale/lidar_range)*randn(size(ranges(ranges < lidar_range)));
    end
    
end

function d_intersect1 = circleLine(p1, p2, center, r)
    % Calculates intersection distance between infinite lines definited by
    % arrays of p1, p2 and a circle of radius r centered at cx, cy
    % d_intersect1 is the closest distance to intersect the circle from p1
    % d_intersect1 -1 means no intersection
    % size of d_intersect1 is 1 x N
    % size of P1 is 2 x N
    % size of P2 is 2 x N
    % Source: https://mathworld.wolfram.com/Circle-LineIntersection.html

    % Normalize coordinates w.r.t. circle center
    x1 = p1(1,:) - center(1);
    y1 = p1(2,:) - center(2);
    x2 = p2(1,:) - center(1);
    y2 = p2(2,:) - center(2);
    
    % Calculates intersection points
    dx = x2 - x1;
    dy = y2 - y1;
    
    dr2 = dx.^2 + dy.^2;
    D = x1.*y2 - x2.*y1;
    discriminant = r^2.*dr2 - D.^2;
    
    xint1 = (D.*dy+sign(dy).*dx.*sqrt(discriminant))./(dr2);
    xint2 = (D.*dy-sign(dy).*dx.*sqrt(discriminant))./(dr2);
    yint1 = (-D.*dx+abs(dy).*sqrt(discriminant))./(dr2);
    yint2 = (-D.*dx-abs(dy).*sqrt(discriminant))./(dr2);
    
    % Calculates intersection distance from point (x1, y1)
    no_intersect = discriminant < 0; % Get indices of non real (no intersect)
    
    diff1 = [x1 - xint1; y1 - yint1];
    diff2 = [x1 - xint2; y1 - yint2];
    d_intersect1 = min(vecnorm(diff1, 2, 1), vecnorm(diff2, 2, 1));
    d_intersect1(no_intersect) = -1;
end


