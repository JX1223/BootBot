function [target, time_until_target] = targetEstimator(s_walker, p_est, v_est)
    % s_walker - 14x1 state of the 5-link walker
    % p_est - 2x1 estimated position of the ball, in world frame
    % v_est - 2x1 estimated velocity of the ball, in world frame
    % target - 2x1 estimated target, in world frame
    % If target is empty, cannot obtain a target that fits requirements
    % Note: controller must be able to reject noisy data, targetEstimator
    % does not account for noise
    constants;
    persistent p_COM_tibia % stores initial swing foot position
    if isempty(p_COM_tibia)
        p_COM_tibia = pComTibia2_gen(s_walker);
    end
    % Parameters
    resolution = 1000; % evaluate <resolution> number of points
    min_height = 0.3; % estimated target should be above this height
    
    % Create a ball trajectory
    % y = y0 + vy0*t - 1/2 gt^2;
    a = p_est(2) - min_height;
    b = v_est(2);
    c = 0.5*g;
    t1 = (-b + sqrt(b^2 - 4 * a * c))/(2*a);
    t2 = (-b - sqrt(b^2 - 4 * a * c))/(2*a);
    if t1 > 0
        time = t1;
    else
        time = t2;
    end
    % Ball is below minimum height, cannot compute a estimated target
    if ~isreal(time) 
        target = [];
        time_until_target = [];
        return;
    end
    t = linspace(0, time, resolution);
    x = p_est(1) + v_est(1).*t;
    y = p_est(2) + v_est(2).*t - (0.5)*g.*t.^2;
    ball = [x; y];    
    
    % Get minimum point on trajectory to swing tibia COM
    distances = ball - p_COM_tibia;
    [min_dist, min_ind] = min(vecnorm(distances, 2, 1));
    target = ball(:, min_ind);
    
    % Get estimated time from "now" until the target
    a = p_est(2) - target;
    b = v_est(2);
    c = 0.5*g;
        t1 = (-b + sqrt(b^2 - 4 * a * c))/(2*a);
    t2 = (-b - sqrt(b^2 - 4 * a * c))/(2*a);
    if t1 > 0
        time_until_target = t1;
    else
        time_until_target = t2;
    end
end