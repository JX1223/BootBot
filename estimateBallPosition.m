function [position, radius] = estimateBallPosition(angles, ranges, constants)
    % Takes in LIDAR readings of angles, ranges and outputs estimated
    % center position of ball in the sensor frame
    % angles is 1 x N, N is number of rays
    % ranges is 1 x N, N is number of rays
    % position is 2 x 1, [x; y]
    % radius is the estimated radius of the ball
    
    % Get constants
    lidar_range = constants.lidar_range;
    
    % First, extract ranged that are not max'd out
    data = ranges < lidar_range;
    angles = angles(data);
    ranges = ranges(data);
    
    % Calculate (x, y) coordinates of measured points on ball circumference
    x = ranges .* cos(angles);
    y = ranges .* sin(angles);
    
    % Estimate circle center
    % Source: https://doi.org/10.1016/0734-189X(89)90088-1
    N = length(x);
    
    if N < 3
        position = [];
        radius = [];
        return
    end
    a1 = 2 * (sum(x)^2 - N * sum(x.^2));
    a2 = 2 * (sum(x)*sum(y) - N * sum(x.*y));
    b1 = 2 * (sum(x)*sum(y) - N * sum(x.*y));
    b2 = 2 * (sum(y)^2 - N * sum(y.^2));
    c1 = (sum(x.^2)*sum(x) - N * sum(x.^3) + sum(x)*sum(y.^2) - N * sum(x.*y.^2));
    c2 = (sum(x.^2)*sum(y) - N * sum(y.^3) + sum(y)*sum(y.^2) - N * sum(x.^2 .* y));
    
    x_est = (c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
    y_est = (a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1);
    position = [x_est; y_est];
    radius = sqrt(1/N * (sum(x.^2) - 2*sum(x)*x_est + N * x_est^2 + sum(y.^2) - 2*sum(y)*y_est + N * y_est^2));
end