function [p_world] = transformBall(s_walker, p_sensor)
    % Transforms measurd ball position in the sensor frame (p_sensor) into
    % the world frame, given knowledge of the state of the walker
    pHead = pHead_gen(s_walker);
    q_torso = s_walker(3);
    if isempty(p_sensor)
        p_world = [];
    else
        p_world = ([cos(-q_torso) -sin(-q_torso); sin(-q_torso) cos(-q_torso)] * p_sensor) + pHead;
    end
end