function [value, isterminal, direction] = forecast2realEvent2(t,x)
    global ballForecast ballForecastTime
    currentBall = interp1(ballForecastTime, ballForecast,t);
    x_robot = x(1:14);
    com = pComTibia2_gen(x_robot);
    dist = sqrt((com(1)-currentBall(1))^2+(com(2)-currentBall(2))^2);
    rad = 1.1/2/2;
    value = dist-rad;
    isterminal = 1;
    direction = 0;
end

