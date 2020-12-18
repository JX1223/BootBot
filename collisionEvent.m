function [position,isterminal,direction] = collisionEvent(t,x)
isterminal = 1;
direction = 1;

target_mode = 'c';
% State of robot
s_robot = x(1:14);

% State of target
s_target = x(15:end);
pSw = pSw_gen(s_robot);
pKnee = pKnee2_gen(s_robot);

switch target_mode
    case 'c'
        l_spring = 1;
        position = -s_robot(1)+(s_target(1)-l_spring);
    case 'p'
        
        pen_pCoM = pen_pCoM_gen(s_target);
        rad = .1;
        dist2Knee = norm(pKnee-pen_pCoM)-rad;
        dist2Foot = norm(pSw - pen_CoM)-rad;
        l_tibia = norm(pKnee-pSw);
        if dist2Knee ==0 || dist2Foot == 0
            position =0;
            
        elseif dist2Foot>l_tibia || dist2Knee>l_tibia
            position = max(dist2Foot,dist2Knee);
        else
            position = abs((pKnee(2)-pSw(2))*pen_pCoM(1) - (pKnee(1)-pSw(1))*pen_pCoM(2) +  pKnee(2)*pSw(1) - pKnee(1)*pSw(2))/...
               sqrt((pKnee(2)-pSw(2))^2 + (pKnee(1)-pSw(1))^2);
        end
        
    otherwise
        position = 1;
end
end

