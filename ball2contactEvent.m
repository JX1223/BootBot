function [value, isterminal, direction] = ball2contactEvent(t,x)
    constants;
    s_rob = x(1:14);
    s_ball = x(15:20);
    pSw = pSw_gen(s_rob);
    pKn = pKnee2_gen(s_rob);
    m = (pSw(2) - pKn(2))/(pSw(1)-pKn(1));
    a = -m;
    b = 1;
    c = m*pSw(1)-pSw(2);
    x0 = s_ball(1);
    y0 = s_ball(2);
    x = (b*(b*x0 - a*y0) - a*c)/(a^2 + b^2);
    y = (a*(-b*x0 + a*y0) - b*c)/(a^2 + b^2);
    dist1 = sqrt((pSw(1)-x)^2 + (pSw(2)-y)^2);
    dist2 = sqrt((pKn(1)-x)^2 + (pKn(2)-y)^2);
    value = (dist1 + dist2 - l_tibia);
    isterminal = 1;
    direction = 0;
end

