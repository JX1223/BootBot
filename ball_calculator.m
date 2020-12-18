function [Jext, Fext] = ball_calculator(t,s)
    ball_radius = 0.03*2;
    s_rob = s(1:14);
    s_ball = s(15:20);
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
    k = 10000000;
    dist = sqrt((x - x0)^2 + (y-y0)^2);
    minX = min(pSw(1),pKn(1));
    minY = min(pSw(2),pKn(2));
    maxX = max(pSw(1),pKn(1));
    maxY = max(pSw(2),pKn(2));
    if (dist <= ball_radius && x >= minX && x<= maxX && y>=minY && y<=maxY)
        mag = min(k*dist,500);
        Fext = [1; -1/m];
        Fext = mag*Fext/norm(Fext);
    else
        Fext = [0;0];
    end
    
    dist2knee = sqrt((x - pKn(1))^2 + (y-pKn(2))^2);

    Jext = Jcontact_gen(s,dist2knee);
end