t = [t1; t2; t3];

[~, ind] = unique(t);
t = t(ind);

s = [s1 interp1(ballForecastTime , ballForecast,t1); 
     s2 interp1(ballForecastTime , ballForecast,t2);
     s3 interp1(ballForecastTime , ballForecast,t3)];
s = s(ind,:);

s_5link = s(:,1:14);
s_ball = s(:,15:20);

pos_ests = zeros(length(t),2);
vel_ests = zeros(length(t),2);

C = constants;

clear ballEstimator
for ii = 1:length(t)
    [pos_est1, vel_est1] = ballEstimator(t(ii),s_5link(ii,:)', s_ball(ii,:)', C);
    if (isempty(pos_est1) || isempty(vel_est1))
        pos_ests(ii,:) = [0,0];
        vel_ests(ii,:) = [0,0];
    else
        pos_ests(ii,:) = pos_est1';
        vel_ests(ii,:) = vel_est1';
    end
end

figure
plot(t,pos_ests(:,1), 'LineWidth', 1);
hold on
plot(t,pos_ests(:,2), 'LineWidth', 1);
plot(t,s(:,15), '--', 'LineWidth', 1)
plot(t,s(:,16), '--', 'LineWidth', 1);
legend({'Est X', 'Est Y', 'Real X', 'Real Y'});
ylabel("Position [m]");
xlabel("Time [s]");
title("Position Estimator, Partially in View");

figure;
plot(t,vel_ests(:,1), 'LineWidth', 1);
hold on
plot(t,vel_ests(:,2), 'LineWidth', 1);
plot(t,s(:,18), '--', 'LineWidth', 1)
plot(t,s(:,19), '--', 'LineWidth', 1);
legend({'Est dX', 'Est dY', 'Real dX', 'Real dY'});
ylabel("Velocity [m/s]");
xlabel("Time [s]");
title("Velocity Estimator, Partially in View");
