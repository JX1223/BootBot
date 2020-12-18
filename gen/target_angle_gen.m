function [kick_angle] = target_angle_gen(s_robot)
bias = 0;
global p_target

hip = s_robot(1:2);

displacement = p_target-hip;

kick_angle = atan(displacement(2)/displacement(1))-bias;

end