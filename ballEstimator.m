function [position, velocity] = ballEstimator(t, s_5link, s_ball, C)
    % Take in current ODE time and state vector (5_link)
    % outputs position and velocity in world frame
    % Position is the LIDAR measured position, but velocity is an estimated
    % velocity
    %
    % There are 4 cases:
    % Case 1: Ball has never been in range
    % Case 2: First measurement of ball in range
    % Case 3: Ball has been in range for at least 2 measurements
    % Case 4: Ball has previously been in range, but went out of range
    
    % Ouputs for each case
    % Case      |       Position Output     |       Velocity Output
    %-------------------------------------------------------------------
    % Case 1    |           EMPTY           |           EMPTY
    % Case 2    |       LIDAR measurement   |           EMPTY
    % Case 3    |       LIDAR measurement   |      velocity estimator
    % Case 4    |      position predictor   |      velocity predictor
    
    % properties
    g = C.g;
    % keeps track of past ball states
    persistent position_prev velocity_prev time_prev t_array v_array;
    
    % Always read lidar in any case
    % Read lidar
    [angles, ranges] = readLIDAR(s_5link, s_ball, C);
    % Estimate position of ball in the sensor frame
    [p_sensor, radius] = estimateBallPosition(angles, ranges, C);
    % Transform position of ball into world frame
    position = transformBall(s_5link, p_sensor);
    
        % Ball has never been in range, or ball has only been in range once
        % -> Case 1 and 2
        if isempty(position_prev)
            position_prev = position;
            velocity = velocity_prev;
            % If previous position is not empty, meaning that this is the first 
            % time the ball is being detected, guess an initial ball velocity.
            if ~isempty(position_prev)
                velocity_prev = [0; 0]; % Assume an initial ball velocity
                v_array = [v_array, [0; 0]];
                t_array = [t_array, t];
            end
            time_prev = t;
            return;
        end

    
    if abs(t - time_prev) >= 0.01

        % Catch if ODE45 decreases the time
        if t < time_prev
            position_prev = position;
            % Erase future time of velocity array "Re-write time"
            future_ind = t_array >= t;
            t_array(future_ind) = [];
            v_array(:,future_ind) = [];
            % Obtain a guess for the current time based on our prev. guesses
            velocity = interp1(t_array', v_array', t, 'linear', 'extrap')';

            % Record previous guesses and extend array
            velocity_prev = v_array(:,end);
            time_prev = t;
            v_array = [v_array, velocity];
            t_array = [t_array, t];
            return;
        end

        % If ball is in range -> Case 3
        if ~isempty(position)
            % Velocity state estimator
            % Differentiate from position measurement
            vel_meas = (position - position_prev) ./ (t - time_prev);

            % Predict ball velocity by modeling as free fall dynamics
            vel_pred = [velocity_prev(1); velocity_prev(2)-g*(t - time_prev)];
            vel_mixing = 0.5; % 0->full measured, 1->full predicted
            velocity = (1-vel_mixing)*vel_meas + vel_mixing*(vel_pred);
            % update stored variables
            position_prev = position;
            velocity_prev = velocity;
            time_prev = t;
            v_array = [v_array, velocity];
            t_array = [t_array, t];
        % If the ball was previously in range, but went out -> Case 4
        else
            % Predict position
            position = position_prev + velocity_prev.*(t - time_prev);
            % Predict velocity
            velocity = [velocity_prev(1); velocity_prev(2)-g*(t - time_prev)];
            % Update stored variables
            position_prev = position;
            velocity_prev = velocity;
            time_prev = t;
            v_array = [v_array, velocity];
            t_array = [t_array, t];
        end
    else
        position = position_prev;
        velocity = velocity_prev;
    end
end