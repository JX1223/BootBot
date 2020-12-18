function [t_ball,s_ball] = forecastBallTrajectory(t_range,initial_ball_state)
%constants;

pass_ball = @(t,s) ballDynamics_externalForce(t,s,[0;0]);
[t_ball, s_ball] = ode45(pass_ball,t_range,initial_ball_state);
end

