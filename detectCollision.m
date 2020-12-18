function collisionDetection = detectCollision(s_robot,s_target,target_mode)

pSw = pSw_gen(s_robot);
pKnee = pKnee2_gen(s_robot);
leg_res = 20;

if leg_res ~=0
    pLegArray = [linspace(pSw(1),pKnee(1),leg_res);
                 linspace(pSw(2),pKnee(2),leg_res)];
end

switch target_mode
    case 'p'
        pen_pCoM = pen_pCoM_gen(s_target);
        rad = .1;
        distances = zeros(leg_res,1);
        if leg_res ~= 0
            for i = 1:leg_res
                distances(i) = (norm(pLegArray(i)-pen_pCoM));
            end
            isCollided = (min(distances) <= rad);
            pCol = pLegArray(:,distances == min(distances));
        else
            isCollided = (norm(pSw-pen_pCoM) <= rad);
            pCol = pSw;
        end
    otherwise
        isCollided = 0;
        pCol = [0;0];
end
collisionDetection = [isCollided;pCol];

end