function [value, isterminal, direction] = forecast2realEvent(t,x)
    global ballForecast ballForecastTime
    currentBall = interp1(ballForecastTime, ballForecast,t);
    x_robot = x(1:14);
    true_state = [x_robot;currentBall'];
    [~, Fext] = ball_calculator(t,true_state);
    value = norm(Fext)-.00001;
    if norm(Fext) > 0
        disp('hey') 
    end
    isterminal = 1;
    direction = 0;
end

