classdef constants
    properties( Constant = true)
        %% Physical Constants
        g = 9.81; % [m/s^2]
        mu_s = 0.8;

        %% Robot Design Parameters
        l_femur = 0.5; % Upper Leg [m]
        l_tibia = 0.5; % Lower Leg [m]
        l_torso = 0.5; % Torso [m]
        m_femur = 5/2;
        m_tibia = 5/2;
        m_torso = 10;
        J_femur = 0;
        J_tibia = 0;
        J_torso = 0;
        m_hip = 15;
        m_total = 2*constants.m_femur + 2*constants.m_tibia + constants.m_torso + constants.m_hip;

        %% Lidar Parameters
        lidar_range = 5; % Lidar range [m]
        lidar_FOV = 2.0944; % Lidar field of view [rads]
        lidar_rays = 300; % Number of lidar rays
        lidar_noiseOn = true; % Turns noise on/off
        noise_scale = 0.01; % Signal-to-Noise ratio

        %% Ball Parameters
        % If you change, also change ballDynamics_externalForce
        ball_radius = 0.03*2; % separately defined in ball_calculator
        m_ball = 3;
        J_ball = 1;

        %% Initial Conditions
        % Knees straight
        % x0 = 0;
        % y0 = l_tibia + l_femur*cos(deg2rad(45));
        % q_torso0 = 0;
        % q1_hip0 = pi/2 + pi/4;
        % q1_knee0 = pi/2 - pi/4;
        % q2_hip0 = 3*pi/2 - pi/4;
        % q2_knee0 = -pi/2 + pi/4;

        % Knees pointed outward
        x0 = 0;
        y0 = constants.l_tibia + constants.l_femur*sqrt(2)/2;
        q_torso0 = 0;
        q1_hip0 = pi/2 + pi/4;
        q1_knee0 = pi/2 - pi/4;
        q2_hip0 = 3*pi/2 - pi/4;
        q2_knee0 = -pi/2 + pi/4;

        % Knees pointed inward
        % x0 = 0;
        % y0 = 0;
        % q_torso0 = 0;
        % q1_hip0 = pi/4;
        % q1_knee0 = pi/2;
        % q2_hip0 = 3*pi/2 + pi/4;
        % q2_knee0 = -pi/2;

        % Knees forward
        % x0 = 0;
        % q_torso0 = 0;
        % q2_hip0 = 4*pi/4;
        % q2_knee0 = pi/4;
        % q1_hip0 = 3*pi/4;
        % q1_knee0 = pi/4;
        % y0 = l_tibia*cos(q1_knee0+q1_hip0-pi+q_torso0)+ l_femur*cos(q1_hip0-pi+q_torso0);

        % Knees forward 2 (kicking stance)
        % x0 = 0;
        % offset = pi/8;
        % inner_angle = 3*pi/8;
        % q_torso0 = 0;
        % q2_hip0 = pi/2+inner_angle+offset;
        % q2_knee0 = pi-2*inner_angle;
        % q1_hip0 = pi/2+inner_angle-offset;
        % q1_knee0 = pi-2*inner_angle;
        % y0 = l_tibia*cos(q1_knee0+q1_hip0-pi+q_torso0)+ l_femur*cos(q1_hip0-pi+q_torso0);

        % % Pre-kick 1
        % x0 = 0;
        % offset = pi/16;
        % y0 = (l_tibia+ l_femur)*cos(offset);
        % q_torso0 = 0;
        % q2_hip0 = pi/4;
        % q2_knee0 = pi/4;
        % q1_hip0 = pi + offset;
        % q1_knee0 = 0;

        % % Pre-kick 2
        % x0 = 0;
        % offset = pi/8;
        % 
        % q_torso0 = 0;
        % q2_hip0 = pi/6;
        % q2_knee0 = pi - q2_hip0;
        % q1_hip0 = pi + pi/8-q_torso0;
        % q1_knee0 = 0;
        % y0 = l_tibia*cos(q1_knee0+q1_hip0-pi+q_torso0)+ l_femur*cos(q1_hip0-pi+q_torso0);

        % Motor Limits

        %% Friction/Damping Coefficients
    end
end
