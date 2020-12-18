function ds = combinedBallDynamics(t,s,C)

constants;
s_rob = s(1:14);
s_ball = s(15:20);

ds_rob = five_link_dynamics_2(t,s_rob);
ds_ball = ballDynamics(t,s_ball);
[pos_est, vel_est] = ballEstimator(t,s);

% global p_target
% 
% if (~isempty(pos_est) && ~isempty(vel_est))
%     [p_target time_est] = targetEstimator(s_rob, pos_est, vel_est);
% end
% p_target

ds = [ds_rob; ds_ball];

end