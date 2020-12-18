%% Ball Kick with break event
addpath('gen');
clear all;
close all;
clc;

global ballForecast ballForecastTime
clear ballEstimator
C = constants;

%initial_ball_state = [1.2;0.6;0;0;0;0]; % Bounce kick
initial_ball_state = [1.1+4.3*0.5;0.6;0;-0.5;0;0]; % Low kick
%initial_ball_state = [1.1+4.3*0.5+0.1;1;0;-0.5;0;0]; % High kick
t_range = [0 10];
[ballForecastTime,ballForecast] = forecastBallTrajectory(t_range,initial_ball_state);
options = odeset('Events',@forecast2realEvent2);

s0 = [C.x0;C.y0;C.q_torso0;C.q1_hip0;C.q1_knee0;C.q2_hip0;C.q2_knee0; ...
    zeros(7,1)];

% Initialize global integral terms
global ie ie_r t_ie controlVersion Fext q_desired kickStart kickEnd lidarSwitch
ie = [0; 0];
ie_r = 0;
t_ie = 0;
controlVersion = 2;
Fext = [0; 0];
kickStart = 0;
kickEnd = 0;
lidarSwitch = 1;

% Simulate and animate
[t1, s1] = ode45(@(t,s) five_link_dynamics_2(t,s,C),[0 3.12],s0,options);
s0 = s1(end,:)';
q_desired = s0(1:7);
controlVersion = 6;

[t2, s2] = ode45(@(t,s)five_link_dynamics_2(t,s,C),[3.12+100*eps,3.5],s0,options);
s0 = s2(end,:)';
q_desired = q_desired + [0; 0; 0; 0; 0; 0; pi/6];

[t3, s3] = ode45(@(t,s)five_link_dynamics_2(t,s,C),[3.5+100*eps,4.6-0.5],s0,options);
s0 = s3(end,:)';
q_desired = [0; 0; 0; C.q1_hip0-.3; C.q1_knee0+.55; C.q2_hip0; C.q2_knee0];
controlVersion = 5;
global p_target t_start s_start pos_est vel_est
t_start = t3(end);
s_start = s0;
p_target = [pos_est(1) + vel_est(1)*0.3; abs(pos_est(2) + vel_est(2)*0.3 - 0.5*C.g*0.3^2)];
p_target = p_target + [0.15; - 0.3];
lidarSwitch = 0;
[t4, s4] = ode45(@(t,s)five_link_dynamics_2(t,s,C),[4.6-0.5+100*eps 5.1-0.5],s0,options);
currentBall = interp1(ballForecastTime , ballForecast,t4(end));
s0 = [s4(end, :)';currentBall'];
controlVersion = 6;
q_desired = s0(1:7) + [0; 0; 0; 0; 0; -0.5; -0.8];
q_desired(7) = 0;

[t4_5, s4_5] = ode45(@(t,s)combinedDynamics(t,s,C),[t4(end)+100*eps 5.1-0.5],s0);
s0 = s4_5(end, :)';
controlVersion = 6;
footCenter = (pSt_gen(s0)+pSw_gen(s0))/2;
q_desired(1:2) = footCenter + [0; 0.75];
q_desired = [q_desired(1:3); s0(4); s0(5)-0.2; q_desired(4)-.45-0.3+0.4;s0(7)-0.6+1.2];
t4(end)

[t5, s5] = ode45(@(t,s)combinedDynamics(t,s,C),[5.1-0.5+100*eps 6.5],s0);

t = [t1; t2; t3; t4; t4_5; t5];

[~, ind] = unique(t);
t = t(ind);

s = [s1 interp1(ballForecastTime , ballForecast,t1); 
     s2 interp1(ballForecastTime , ballForecast,t2);
     s3 interp1(ballForecastTime , ballForecast,t3);
     s4 interp1(ballForecastTime , ballForecast,t4);
     s4_5;
     s5];
 s = s(ind,:);
 
f = animateFiveLinkInterp_withBall(t,s,60, C);
