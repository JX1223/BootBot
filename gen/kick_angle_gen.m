function [kick_angle] = kick_angle_gen(s_robot)

pSw = pSw_gen(s_robot);
hip = s_robot(1:2);

displacement = pSw-hip;

kick_angle = atan(displacement(2)/displacement(1));

end

