function u = controller(t,x,C)
%% Control Version
global controlVersion q_desired

switch controlVersion
    
%% Nominal PD Control
case 0
    %constants;
    n = NDoF_gen(x);
    q = x(1:n);
    dq = x(n+1:end);
    q1_hip = q(4);
    q1_knee = q(5);
    q2_hip = q(6);
    q2_knee = q(7);
    dq1_hip = dq(4);
    dq1_knee = dq(5);
    dq2_hip = dq(6);
    dq2_knee = dq(7);
    q1_hip_d = q1_hip0;
    q1_knee_d = q1_knee0;
    q2_hip_d = q2_hip0;
    q2_knee_d = q2_knee0;
    k1 = -1000; k2 = -500; k3 = -1000; k4 = -200;
    dk1 = -20; dk2 = -20; dk3 = -70; dk4 = -30;


    u = [k1*(q1_hip-q1_hip_d) + dk1*dq1_hip; ...
        k2*(q1_knee-q1_knee_d) + dk2*dq1_knee; ...
        k3*(q2_hip-q2_hip_d) + dk3*dq2_hip; ...
        k4*(q2_knee-q2_knee_d) + dk4*dq2_knee];
%% Gain Scheduled PD
case 1
    %constants;
    n = NDoF_gen(x);
    q = x(1:n);
    dq = x(n+1:end);
    q1_hip = q(4);
    q1_knee = q(5);
    q2_hip = q(6);
    q2_knee = q(7);
    dq1_hip = dq(4);
    dq1_knee = dq(5);
    dq2_hip = dq(6);
    dq2_knee = dq(7);
    q1_hip_d = q1_hip0;
    q1_knee_d = q1_knee0;
    
    q2_hip_d = q2_hip0;
    q2_knee_d = q2_knee0;
    k1 = -1000; k2 = -500; k3 = -1000; k4 = -200;
    dk1 = -20; dk2 = -20; dk3 = -70; dk4 = -30;
%% get to kick position
    if t > .05
        q2_hip_d = pi+pi/3;
        q2_knee_d = 0;
        q1_knee_d = pi/8;
        q1_hip_d = 3*pi/4;
        
%         if t > .1
%              q1_knee_d = pi/8;
%              q1_hip_d = 3*pi/4;
%              q2_hip_d = pi+pi/8;
%             q2_knee_d = pi/4;
%         end
        k1 = -700; k2 = -600; k3 = -1200; k4 = -1200;
        dk1 = -200; dk2 = -200; dk3 = -200; dk4 = -200;
    end
    
    if t>.3
        q1_hip_d = pi+pi/6;
        q1_knee_d = pi/6;
        q2_hip_d = pi/6;
        q2_knee_d = pi - q2_hip_d;
        k1 = -900; k2 = -900; k3 = -600; k4 = -700;
        dk1 = -100; dk2 = -100; dk3 = -80; dk4 = -50;
    end
%% hard coded kick, cart at 1.5, still    
%     if t > .05
%         q1_hip_d = pi + pi/4;
%         q1_knee_d =0;
%         q2_hip_d = 3*pi/4;
%         q2_knee_d = 0;
%     end
%     k1 = -5000; k2 = -5000; k3 = -1000; k4 = -1500;
%     dk1 = -200; dk2 = -200; dk3 = -70; dk4 = -30;
%     
%     if t>.2
%         q1_hip_d = pi+pi/8 ;
%         q1_knee_d = pi/8;
%         q2_hip_d = pi - pi/8;
%         q2_knee_d = pi/8;
%         k1 = -1000; k2 = -500; k3 = -1000; k4 = -200;
%         dk1 = -20; dk2 = -20; dk3 = -70; dk4 = -30;
%     end


%%
%     if t>.35
%         q1_hip_d = pi-1/3*pi ;
%         q1_knee_d = 0/8*pi;
%         q2_hip_d = pi +1/6*pi;
%         q2_knee_d = 1/6*pi;
%         k1 = -500; k2 = -500; k3 = -500; k4 = -500;
%         dk1 = -20; dk2 = -20; dk3 = -70; dk4 = -30;
%     end
    
%     if t>.5
%         q1_hip_d = pi-1/3*pi ;
%         q1_knee_d = 1/3*pi;
%         q2_hip_d = pi +1/6*pi;
%         q2_knee_d = 1/6*pi;
%         k1 = -500; k2 = -500; k3 = -500; k4 = -500;
%         dk1 = -20; dk2 = -20; dk3 = -70; dk4 = -30;
%     end
    u = [k1*(q1_hip-q1_hip_d) + dk1*dq1_hip; ...
        k2*(q1_knee-q1_knee_d) + dk2*dq1_knee; ...
        k3*(q2_hip-q2_hip_d) + dk3*dq2_hip; ...
        k4*(q2_knee-q2_knee_d) + dk4*dq2_knee];
    u = u.*[1;1;1;1];
%% Testing
case 1.5
    %constants;

    global kickStart kickEnd mcart p_target
    n = NDoF_gen(x);
    q = x(1:n);
    dq = x(n+1:end);
    q1_hip = q(4);
    q1_knee = q(5);
    q2_hip = q(6);
    q2_knee = q(7);
    dq1_hip = dq(4);
    dq1_knee = dq(5);
    dq2_hip = dq(6);
    dq2_knee = dq(7);
    q1_hip_d = q1_hip0;
    q1_knee_d = q1_knee0;
    
    q2_hip_d = q2_hip0;
    q2_knee_d = q2_knee0;
    k1 = -1000; k2 = -500; k3 = -1000; k4 = -200;
    dk1 = -20; dk2 = -20; dk3 = -70; dk4 = -30;
%% get to kick position
    timestart = .05;
    if t > timestart
        offset2 = pi/8;
        offset1 = pi/8;
        inner_angle_1 = 1/4*pi;
        inner_angle_2 = 3/8*pi;
        q2_hip_d = pi/2+inner_angle+offset2;
        q2_knee_d = pi-2*inner_angle_2;
        q1_hip_d = pi/2+inner_angle-offset1;
        q1_knee_d = pi-2*inner_angle_1;

        k1 = -600; k2 = -600; k3 = -1200; k4 = -1200;
        dk1 = -200; dk2 = -200; dk3 = -200; dk4 = -200;
    end
    
    if t > .4+.1
        torso_angle = pi/8;
        offset2 = pi/2;
%         offset1 = pi/8;
        offset1 = -1*pi/8;
        

        inner_angle_1 = 1/3*pi;
        inner_angle_2 = 1/8*pi;
%         q2_hip_d = pi/2+inner_angle+offset2;
        q2_hip_d = pi/2+inner_angle-offset2;
        q2_knee_d = pi-2*inner_angle_2;
        q1_hip_d = pi/2+inner_angle-offset1+torso_angle;
        q1_knee_d = pi-2*inner_angle_1;
        
        offset2 = 3*pi/4;
        inner_angle_2 = 1/8*pi;
        q2_hip_d = pi/2+inner_angle-offset2+torso_angle;
        q2_knee_d = pi-2*inner_angle_2;
% 
%         k1 = -700; k2 = -600; k3 = -1200; k4 = -1200;
%         dk1 = -200; dk2 = -200; dk3 = -200; dk4 = -50;
        k1 = -700; k2 = -1200; k3 = -1200; k4 = -1200;
        dk1 = -100; dk2 = -400; dk3 = -200; dk4 = -50;
    end
    %was .7
%     if t > .6
%         
%         offset1 = pi/3;
%         inner_angle_1 = 1/4*pi;
%         offset2 = 3*pi/4;
%         inner_angle_2 = 1/8*pi;
%         q2_hip_d = pi/2+inner_angle-offset2;
%         q2_knee_d = pi-2*inner_angle_2;
%         q1_hip_d = pi/2+inner_angle+offset1;
%         q1_knee_d = pi-2*inner_angle_1;
% 
% %         k1 = -1200; k2 = -1200; k3 = -1200; k4 = -1200;
% %         dk1 = -200; dk2 = -200; dk3 = -200; dk4 = -50;
%         k1 = -1200; k2 = -1200; k3 = -1200; k4 = -1200;
%         dk1 = -200; dk2 = -500; dk3 = -200; dk4 = -50;
%     end
    
%     if t>.3
%         q1_hip_d = pi+pi/6;
%         q1_knee_d = pi/6;
%         q2_hip_d = pi/6;
%         q2_knee_d = pi - q2_hip_d;
%         k1 = -900; k2 = -900; k3 = -600; k4 = -700;
%         dk1 = -100; dk2 = -100; dk3 = -80; dk4 = -50;
%     end
%% hard coded kick, cart at 1.5, still    
%     if t>1.15
    if t>.85+.1 -.05 - .05  %<---- this one works

        torso_angle = pi/4;


        if p_target(2)<0
            target_angle = 0;
        else
            target_angle = target_angle_gen(x);
        end
%         kick_angle = rad2deg(kick_angle_gen(q))

%         target_angle = 0;
%         kick_angle = rad2deg(kick_angle_gen(q))
        q1_hip_d = pi + torso_angle -pi/8;%messing around
        q1_knee_d =pi/16;%pi/16-pi/16+1/3*pi;
        q2_hip_d = pi/2;
        q2_hip_d = 2*pi/4 + torso_angle;
        q2_hip_d = pi/2-q(3)-q2_knee/2-target_angle;
        q2_knee_d = 0;
        k1 = -1000; k2 = -1000; k3 = -1000; k4 = -1000;
        dk1 = -200; dk2 = -200; dk3 = -70; dk4 = -30;
        dk1 = -200; dk2 = -200; dk3 = -30; dk4 = -30;
        dk1 = -30; dk2 = -100; dk3 = -30; dk4 = -30;

        dk1 = -30; dk2 = -200+80; dk3 = -50; dk4 = -50 * max((mcart/10),1);
        k3 = -5000-2000;
        
        if abs(q2_knee)<pi/8
            q2_hip_d = max(pi/2+pi/6,q2_hip_d);
            k3 = -2000;
        end
        if kickStart ==1 && kickEnd ==1
            q1_hip_d = pi + -q(3);
            k2 = 0;
        end
        k3 = k3-3000;
        dk4 = -100 * max((mcart/10),1);
        dk4 = -50 * max((mcart/10),1); % rational

        q2_hip_d = max(pi/6 , min(7*pi/8, q2_hip_d));
    end
    
    
%     if t>1.3
    landing = 0;
    if t>1%1.1%1.2%.95
        q1_hip_d = pi+pi/8 ;
        q1_knee_d = pi/8;
        q2_hip_d = pi - pi/4;
        q2_knee_d = pi/4;
        k1 = -500; k2 = -500; k3 = -500; k4 = -500;
        dk1 = -200; dk2 = -100; dk3 = -200; dk4 = -100;
        
        % Messing around to get softer landing
        k1 = -1000; k2 = -500; k3 = -1000; k4 = -500;
        dk1 = -500; dk2 = -50; dk3 = -50; dk4 = -50;
        landing = 1;
    end

    pf1 = pFoot1_gen(q);
    pf2 = pFoot2_gen(q);
    
    threshfoot = .0;
%     if pf1(2)<=threshfoot && pf2(2)<=threshfoot && landing ==1
%         controlVersion = 2;
%     end
    

%%
%     if t>.35
%         q1_hip_d = pi-1/3*pi ;
%         q1_knee_d = 0/8*pi;
%         q2_hip_d = pi +1/6*pi;
%         q2_knee_d = 1/6*pi;
%         k1 = -500; k2 = -500; k3 = -500; k4 = -500;
%         dk1 = -20; dk2 = -20; dk3 = -70; dk4 = -30;
%     end
    
%     if t>.5
%         q1_hip_d = pi-1/3*pi ;
%         q1_knee_d = 1/3*pi;
%         q2_hip_d = pi +1/6*pi;
%         q2_knee_d = 1/6*pi;
%         k1 = -500; k2 = -500; k3 = -500; k4 = -500;
%         dk1 = -20; dk2 = -20; dk3 = -70; dk4 = -30;
%     end
    u = [k1*(q1_hip-q1_hip_d) + dk1*dq1_hip; ...
        k2*(q1_knee-q1_knee_d) + dk2*dq1_knee; ...
        k3*(q2_hip-q2_hip_d) + dk3*dq2_hip; ...
        k4*(q2_knee-q2_knee_d) + dk4*dq2_knee];
    u = u.*[1;1;1;1];
    
    % Saturator
    mt = 500;
    for i = 1:4
        u(i) = max(-mt,min(u(i),mt));
    end
    
%% Contact Force Optimization
case 2
    %constants;
    global ie ie_r t_ie
    n = NDoF_gen(x);
    q = x(1:n);
    dq = x(n+1:end);
    
    % Initial conditions
    q0 = [C.x0; C.y0; C.q_torso0; C.q1_hip0; C.q1_knee0; C.q2_hip0; C.q2_knee0];
    
    % Compute positional force wrench
    r = robot_CoM_gen(x, C);
    r_des = robot_CoM_gen(q0, C);
    dr = robot_dCoM_gen(x);
    dr_des = [0; 0]; % magic number!
    
    %r_des = [q_desired(1);q_desired(2)]; %CCC trying out to define COM position
    
    r_des = r_des + (t > 2)*[0.1; 0];

    kp = [300 0; ...
          0 300];
    kd = [100 0; ...
          0 100];
    ki = [100 0; ...
          0 100];
      
    %Jared Messing with it
%     r_des(1) = r(1);
%     r_des(2) = r_des(2)-.1;
    
    e = r - r_des;
    de = dr - dr_des;
    if (t > t_ie)
        ie = ie + e*(t-t_ie);
    end
    f_b = -kp*(e) -kd*(de) -ki*(ie);
    f_f = [0; C.m_total*C.g*0.75];
    f_GA = -f_b - f_f;
    
   
    % Compute rotational wrench
    th = pi/2 - q(3); % converting to horizontal angle reference
    th_des = pi/2 - C.q_torso0;
    omega = -dq(3); % converting to horizontal angle reference
    k_r = 50;
    k_damp = 20;
    k_integral = 20;
    e_r = th-th_des;
    if (t > t_ie)
        ie_r = ie_r + e_r*(t-t_ie);
        t_ie = t;
    end
    tau_GA = k_r*(e_r) + k_damp*omega + k_integral*ie_r;
    
    % Combine into wrench
    F_GA = [f_GA; tau_GA];
    
    % Compute grasp map
    pF1 = pFoot1_gen(x) - r;
    pF2 = pFoot2_gen(x) - r;
    G_C = [1 0 1 0; ...
        0 1 0 1; ...
        -pF1(2) pF1(1) -pF2(2) pF2(1)];
    
    % Optimization
    n_fc = 4;
    AAeq = G_C;
    bbeq = F_GA;
    mu_s = C.mu_s;
    AA = [1 mu_s 0 0; ...
          0 0  1 mu_s; ...
         -1 mu_s 0 0; ...
          0 0 -1 mu_s];
    bb = zeros(4,1);
    HH = [ 5 0 0 0 ;
           0 1 0 0 ;
           0 0 5 0 ; 
           0 0 0 1 ];
    ff = zeros(n_fc,1);
    l_b = -10000*ones(n_fc,1);
    u_b = [10000; 0; 10000; 0];
    xx0 = [0; -C.m_total*C.g/2; 0; -C.m_total*C.g/2];
    options = optimset('Display', 'off');
    f_c = quadprog(HH,ff,AA,bb,AAeq,bbeq,l_b,u_b,xx0,options);
    tau_passer = @(f) tauTotal(f,x);
    diff_between = 0;
    f_c = fmincon(tau_passer , xx0,AA,bb,AAeq,bbeq,l_b,u_b,@faux_function ,options);
    
    
    
    if isempty(f_c)
        f_c = xx0;
        disp(t)
    end
    
    % Compute motor torques
    %f_c = [0; -m_total*g/2; 0; -m_total*g/2];
    tau = JSt2_gen(x)'*f_c;
    u = tau(4:end);
    
case 3
    %constants;
    n = NDoF_gen(x);
    q = x(1:n);
    dq = x(n+1:end);
    
    % Initial conditions
    q0 = [x0; y0; q_torso0; q1_hip0; q1_knee0; q2_hip0; q2_knee0];
    
    % Compute positional force wrench
    r = robot_CoM_gen(x);
    r_des = robot_CoM_gen(q0)+[0;0];
    dr = robot_dCoM_gen(x);
    dr_des = [0; 0]; % magic number!
    kp = [500 0; ...
          0 500];
    kd = [100 0; ...
          0 100];
    f_b = -kp*(r - r_des) -kd*(dr - dr_des);
    f_f = [0; C.m_total*C.g];
    f_GA = -f_b - f_f;
    
    % Compute rotational wrench
    th = pi/2 - q(3); % converting to horizontal angle reference
    th_des = pi/2 - q_torso0;
    omega = -dq(3); % converting to horizontal angle reference
    k_r = 50;
    k_damp = 20;
    tau_GA = k_r*(th-th_des) + k_damp*omega;
    
    % Combine into wrench
    F_GA = [f_GA; tau_GA];
    
    % Compute grasp map
    pF1 = pFoot1_gen(x) - r;
    pF2 = pFoot2_gen(x) - r;
    G_C = [1 0 1 0; ...
        0 1 0 1; ...
        -pF1(2) pF1(1) -pF2(2) pF2(1)];
    G_C = [1 0 ; ...
        0 1 ; ...
        -pF1(2) pF1(1)];
    % Optimization
    n_fc = 4;
    AAeq = G_C;
    bbeq = F_GA;
    mu_s = C.mu_s;
    AA = [1 mu_s; ...
%           0 0  1 mu_s; ...
         -1 mu_s]; ...
%           0 0 -1 mu_s];
    bb = zeros(n_fc,1);
    HH = [ 5 0 0 0 ;
           0 1 0 0 ;
           0 0 5 0 ; 
           0 0 0 1 ];
    ff = zeros(n_fc,1);
    l_b = -10000*ones(n_fc,1);
    u_b = [10000; 0];
%     xx0 = [0; -m_total*g/2; 0; -m_total*g/2];
    xx0 = [0; -C.m_total*C.g];
    options = optimset('Display', 'off');
    %f_c = quadprog(HH,ff,AA,bb,AAeq,bbeq,l_b,u_b,xx0,options);
    tau_passer = @(f) tauTotal(f,x);
    diff_between = 0;
    f_c = fmincon(tau_passer , xx0,AA,bb,AAeq,bbeq,l_b,u_b,@faux_function ,options);
      
    if isempty(f_c)
        f_c = xx0;
        disp(t)
    end
      
    % Compute motor torques
    %f_c = [0; -m_total*g/2; 0; -m_total*g/2];
    f_c = [f_c;40;40];
    tau = JSt2_gen(x)'*f_c ;
    u = [tau(4:end)];
%% slow move
case 4
    %constants;
    n = NDoF_gen(x);
    q = x(1:n);
    dq = x(n+1:end);
    
    % Initial conditions
    q0 = [x0; y0; q_torso0; q1_hip0; q1_knee0; q2_hip0; q2_knee0];
    
    % Compute positional force wrench
    r = robot_CoM_gen(x)
    r_des = robot_CoM_gen(q0)+ [.3*t;.08*t];
    dr = robot_dCoM_gen(x) ;
    dr_des = [0; 0]; % magic number!
    kp = [500 0; ...
          0 500];
    kd = [100 0; ...
          0 100];
      kd = [10 0; ...
          0 100];
    f_b = -kp*(r - r_des) -kd*(dr - dr_des);
    f_f = [0; C.m_total*C.g];
    f_GA = -f_b - f_f;
    
    % Compute rotational wrench
    th = pi/2 - q(3); % converting to horizontal angle reference
    th_des = pi/2 - q_torso0;
    omega = -dq(3); % converting to horizontal angle reference
    k_r = 50;
    k_damp = 20;
    tau_GA = k_r*(th-th_des) + k_damp*omega;
    
    % Combine into wrench
    F_GA = [f_GA; tau_GA];
    
    % Compute grasp map
    pF1 = pFoot1_gen(x) - r;
    pF2 = pFoot2_gen(x) - r;
    G_C = [1 0 1 0; ...
        0 1 0 1; ...
        -pF1(2) pF1(1) -pF2(2) pF2(1)];
    
    % Optimization
    n_fc = 4;
    AAeq = G_C;
    bbeq = F_GA;
    mu_s = C.mu_s;
    AA = [1 mu_s 0 0; ...
          0 0  1 mu_s; ...
         -1 mu_s 0 0; ...
          0 0 -1 mu_s];
    bb = zeros(4,1);
    HH = [ 5 0 0 0 ;
           0 1 0 0 ;
           0 0 5 0 ; 
           0 0 0 1 ];
    ff = zeros(n_fc,1);
    l_b = -10000*ones(n_fc,1);
    u_b = [10000; 0; 10000; 0];
    xx0 = [0; -C.m_total*C.g/2; 0; -C.m_total*C.g/2];
    options = optimset('Display', 'off');
    f_c = quadprog(HH,ff,AA,bb,AAeq,bbeq,l_b,u_b,xx0,options);
    tau_passer = @(f) tauTotal(f,x);
    diff_between = 0;
    f_c = fmincon(tau_passer , xx0,AA,bb,AAeq,bbeq,l_b,u_b,@faux_function ,options);
    
    
    
    if isempty(f_c)
        f_c = xx0;
        disp(t)
    end
    
    
    
    % Compute motor torques
    %f_c = [0; -m_total*g/2; 0; -m_total*g/2];
    tau = JSt2_gen(x)'*f_c;
    u = tau(4:end);
    
case 5  %TASK SPACE CONTROLLER
    %constants;
    n = NDoF_gen(x);
    q = x(1:n);
    dq = x(n+1:end);
    
    % Initial conditions
    pSw = pSw_gen(x);  % current position of end effector
    dpSw = dpSw_gen(x); % current velocity of end effector
    %pSw = robot_CoM_gen_2(x);
    %dpSw = robot_dCoM_gen(x);
    omega1 = 0;  %omega_hip
    omega2 = 0;  %omega_knee
    q0 = [C.x0; C.y0; C.q_torso0; C.q1_hip0-0.5; C.q1_knee0+0.5; C.q2_hip0; C.q2_knee0;0;0;0;0;0;0;0];
    %x_des = q0 + [0;0;0;0;0; omega1*t; omega2*t;0;0;0;0;0;omega1; omega2];
    t_con = 8;
    x_mag = 1.2;
    y_mag = 0.7;
    q_des = [x_mag*sin(t_con*t)*(t<pi/2/t_con) + x_mag*(t>=pi/2/t_con);y_mag*sin(t_con*t)*(t<pi/2/t_con) + y_mag*(t>=pi/2/t_con)];%.064286];%pSw_gen(x_des);  %q_des = desired position of the end effector
                             %dq_des = desired velocity of the end effector
    dq_des = [x_mag*t_con*cos(t_con*t)*(t<pi/2/t_con);y_mag*t_con*cos(t_con*t)*(t<pi/2/t_con)];%[omega1*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t) + (omega1+omega2)*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t-q2_knee0-omega2*t);...
              %-omega1*.5*cos(pi/2-q_torso0-q2_hip0-omega1*t) - (omega1+omega2)*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t-q2_knee0-omega2*t)];
    
    t_con = 0.3;
    global p_target t_start s_start
    pSw0 = pSw_gen(s_start);
    px0 = pSw0(1);
    py0 = pSw0(2);
    if (t < t_start + t_con)
        q_des = [px0 + (t-t_start)/t_con*(p_target(1)-px0); py0 + (t-t_start)/t_con*(p_target(2)-py0)];
        dq_des = [1/t_con*(p_target(1)-px0); 1/t_con*(p_target(2)-py0)];
    else
        q_des = [p_target(1); p_target(2)];
        dq_des = [0; 0];
        %q_des = pSw;
        %dq_des = [0.6/0.2; -0.6/0.2];
    end
    
    A = D_gen(x);
    B = C_gen(x)*dq;
    G = G_gen(x);
    J = JSw_gen(x);  % jacobian for end effector
    %J = JCoM_gen(x);
    lambda = inv(J * inv(A) * J');
    Jbar = inv(A) * J' * lambda;
    mu = Jbar' * B - lambda * dJSw_gen(x) * dq;
    %mu = Jbar' * B - lambda * dJCoM_gen(x) * dq;
    p = Jbar' * G;
    kp = 45+30; kd = 20; % gains for this controller
    term = lambda * (-kp*(pSw - q_des) - kd*(dpSw -dq_des));
    F = mu + p + term;
    % null space term
    c_des = [0; 0.064286];
    dc_des = [0; 0];
    kp_c = 10; kd_c = 10;
    kp_c = 100; kd_c = 100; % Jared testing
    N = eye(n) - Jbar*J;
    pC = robot_CoM_gen_2(x);
    dpC = robot_dCoM_gen(x);
    isolated_q = [q(1:5);q0(6:7)]; %jared testing
    isolated_dq = [dq(1:5);0;0]; % Jared Testing
    null_term = -kp_c*(q - q0(1:n)) -kd_c*(dq);%B + G + A*(-kd_c*(q - q0(1:n)));
    null_term = -kp_c*(isolated_q - q0(1:n)) -kd_c*(isolated_dq);%B + G + A*(-kd_c*(q - q0(1:n)));
%     null_term = zeros(7,1);
    tau = J' * F + N'*null_term;
    u = tau(4:end);

%     Jtor = 
    
% Trying to get stance foot
case 6
    %constants;
    n = NDoF_gen(x);
    q = x(1:n);
    dq = x(n+1:end);
    q1_hip = q(4);
    q1_knee = q(5);
    q2_hip = q(6);
    q2_knee = q(7);
    dq1_hip = dq(4);
    dq1_knee = dq(5);
    dq2_hip = dq(6);
    dq2_knee = dq(7);
    q1_hip_d = q_desired(4);
    q1_knee_d = q_desired(5);
    q2_hip_d = q_desired(6);
    q2_knee_d = q_desired(7);
    k1 = -1000; k2 = -500; k3 = -1000; k4 = -200;
    dk1 = -20; dk2 = -20; dk3 = -70; dk4 = -30;


    u = [k1*(q1_hip-q1_hip_d) + dk1*dq1_hip; ...
        k2*(q1_knee-q1_knee_d) + dk2*dq1_knee; ...
        k3*(q2_hip-q2_hip_d) + dk3*dq2_hip; ...
        k4*(q2_knee-q2_knee_d) + dk4*dq2_knee];
    
    
case 7  %TASK SPACE CONTROLLER 2
    %constants;
    n = NDoF_gen(x);
    q = x(1:n);
    dq = x(n+1:end);
    
    % Initial conditions
    pSw = pSw_gen(x);  % current position of end effector
    dpSw = dpSw_gen(x); % current velocity of end effector
    %pSw = robot_CoM_gen_2(x);
    %dpSw = robot_dCoM_gen(x);
    omega1 = 0;  %omega_hip
    omega2 = 0;  %omega_knee
    q0 = [C.x0; C.y0; C.q_torso0; C.q1_hip0-0.5; C.q1_knee0+0.5; C.q2_hip0; C.q2_knee0;0;0;0;0;0;0;0];
    %x_des = q0 + [0;0;0;0;0; omega1*t; omega2*t;0;0;0;0;0;omega1; omega2];
    t_con = 8;
    x_mag = 1.2;
    y_mag = 0.7;
    t_con = 10; % Jared testing
    x_mag = 1.2; % Jared testing
    y_mag = 0.5; % Jared testing
    q_des = [x_mag*sin(t_con*t)*(t<pi/2/t_con) + x_mag*(t>=pi/2/t_con);y_mag*sin(t_con*t)*(t<pi/2/t_con) + y_mag*(t>=pi/2/t_con)];%.064286];%pSw_gen(x_des);  %q_des = desired position of the end effector
                             %dq_des = desired velocity of the end effector
    dq_des = [x_mag*t_con*cos(t_con*t)*(t<pi/2/t_con);y_mag*t_con*cos(t_con*t)*(t<pi/2/t_con)];%[omega1*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t) + (omega1+omega2)*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t-q2_knee0-omega2*t);...
              %-omega1*.5*cos(pi/2-q_torso0-q2_hip0-omega1*t) - (omega1+omega2)*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t-q2_knee0-omega2*t)];
    
              
    inner_angle_0 = (pi-q0(7))/2;
    inner_angle = (pi-q(7))/2;
    from_H_0 = q0(6)-inner_angle_0;
    inner_angle_f =pi/10;
    global p_target
    if p_target(2)<0
        target_angle = 0;
    else
        target_angle = target_angle_gen(x);
    end
    t_tot_d = .3;
    
    inner_angle_des = inner_angle_0 + (inner_angle_f - inner_angle_0)/t_tot_d * min(t,t_tot_d);
    d_inner_angle_des = (inner_angle_f - inner_angle_0)/t_tot_d;
    if t > t_tot_d
        inner_angle_f = pi/2*.9;
        inner_angle_des = inner_angle_0 + (inner_angle_f - inner_angle_0)/t_tot_d * min(t-t_tot_d,t_tot_d);
        d_inner_angle_des = (inner_angle_f - inner_angle)/.03;
    end
    d_from_H_des = (-target_angle - from_H_0)/t_tot_d;
    if t > t_tot_d
        d_from_H_des = target_angle-kick_angle_gen(x);
    end
    from_H_des = from_H_0 + (-target_angle - from_H_0)/t_tot_d  * min(t,t_tot_d);
    
    q_des = [q(1);q(2)]+2*l_tibia*[sin(inner_angle_des)*cos(-from_H_des);sin(inner_angle_des)*sin(-from_H_des)];
    dq_des = [dq(1);dq(2)]+2*l_tibia*[d_inner_angle_des*cos(inner_angle_des)*cos(-from_H_des) + d_from_H_des*sin(inner_angle_des)*sin(-from_H_des);
                                      d_inner_angle_des*cos(inner_angle_des)*sin(-from_H_des) - d_from_H_des*sin(inner_angle_des)*cos(-from_H_des)];
              
    A = D_gen(x);
    B = C_gen(x)*dq;
    G = G_gen(x);
    J = JSw_gen(x);  % jacobian for end effector
    %J = JCoM_gen(x);
    lambda = inv(J * inv(A) * J');
    Jbar = inv(A) * J' * lambda;
    mu = Jbar' * B - lambda * dJSw_gen(x) * dq;
    %mu = Jbar' * B - lambda * dJCoM_gen(x) * dq;
    p = Jbar' * G;
    kp = 500; kd = 500; % gains for this controller
    term = lambda * (-kp*(pSw - q_des) - kd*(dpSw -dq_des));
%     if (t > pi/2/t_con+0.1)
%         pSw-q_des
%         controlVersion = 6;
%         q_desired = q;
%     end
    F = mu + p + term;
    % null space term
    c_des = [0; 0.064286];
    dc_des = [0; 0];
    kp_c = 10; kd_c = 10;
    kp_c = 1000; kd_c = 100; % Jared testing
    N = eye(n) - Jbar*J;
    pC = robot_CoM_gen_2(x);
    dpC = robot_dCoM_gen(x);
    isolated_q = [q(1:5);q0(6:7)]; %jared testing
    isolated_dq = [dq(1:5);0;0]; % Jared Testing
    null_term = -kp_c*(q - q0(1:n)) -kd_c*(dq);%B + G + A*(-kd_c*(q - q0(1:n)));
    null_term = -kp_c*(isolated_q - q0(1:n)) -kd_c*(isolated_dq);%B + G + A*(-kd_c*(q - q0(1:n)));
%     null_term = zeros(7,1);
    tau = J' * F + N'*null_term;
    u = tau(4:end);

%     Jtor = 
    
% Trying to get stance foot
case 7.5  %TASK SPACE CONTROLLER 2
    %constants;
    n = NDoF_gen(x);
    q = x(1:n);
    dq = x(n+1:end);
    
    % Initial conditions
    pSw = pSw_gen(x);  % current position of end effector
    dpSw = dpSw_gen(x); % current velocity of end effector
    pSt = pSt_gen(x);  % current position of end effector
    dpSt = dpSt_gen(x); % current velocity of end effector
    %pSw = robot_CoM_gen_2(x);
    %dpSw = robot_dCoM_gen(x);
    omega1 = 0;  %omega_hip
    omega2 = 0;  %omega_knee
    q0 = [C.x0; C.y0; C.q_torso0; C.q1_hip0-0.5; C.q1_knee0+0.5; C.q2_hip0; C.q2_knee0;0;0;0;0;0;0;0];
        q0 = [C.x0; C.y0; C.q_torso0; C.q1_hip0; C.q1_knee0; C.q2_hip0; C.q2_knee0;0;0;0;0;0;0;0];

    %x_des = q0 + [0;0;0;0;0; omega1*t; omega2*t;0;0;0;0;0;omega1; omega2];
    t_con = 8;
    x_mag = 1.2;
    y_mag = 0.7;
    t_con = 10; % Jared testing
    x_mag = 1.2; % Jared testing
    y_mag = 0.5; % Jared testing
    q_des = [x_mag*sin(t_con*t)*(t<pi/2/t_con) + x_mag*(t>=pi/2/t_con);y_mag*sin(t_con*t)*(t<pi/2/t_con) + y_mag*(t>=pi/2/t_con)];%.064286];%pSw_gen(x_des);  %q_des = desired position of the end effector
                             %dq_des = desired velocity of the end effector
    dq_des = [x_mag*t_con*cos(t_con*t)*(t<pi/2/t_con);y_mag*t_con*cos(t_con*t)*(t<pi/2/t_con)];%[omega1*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t) + (omega1+omega2)*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t-q2_knee0-omega2*t);...
              %-omega1*.5*cos(pi/2-q_torso0-q2_hip0-omega1*t) - (omega1+omega2)*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t-q2_knee0-omega2*t)];
    
              
    inner_angle_0 = (pi-q0(7))/2;
    inner_angle = (pi-q(7))/2;
    from_H_0 = q0(6)-inner_angle_0;
    inner_angle_f = pi/10;
    global p_target
    if p_target(2)<0
        target_angle = 0;
    else
        target_angle = target_angle_gen(x);
    end
    t_tot_d = .3;
    
    inner_angle_des = inner_angle_0 + (inner_angle_f - inner_angle_0)/t_tot_d * min(t,t_tot_d);
    d_inner_angle_des = (inner_angle_f - inner_angle_0)/t_tot_d;
    if t > t_tot_d
        inner_angle_f = pi/2*.9;
        inner_angle_des = inner_angle_0 + (inner_angle_f - inner_angle_0)/t_tot_d * min(t-t_tot_d,t_tot_d);
        d_inner_angle_des = (inner_angle_f - inner_angle)/.03;
    end
    d_from_H_des = (-target_angle - from_H_0)/t_tot_d;
    if t > t_tot_d
        d_from_H_des = target_angle-kick_angle_gen(x);
    end
    from_H_des = from_H_0 + (-target_angle - from_H_0)/t_tot_d  * min(t,t_tot_d);
    
    q_des = [q(1);q(2)]+2*l_tibia*[sin(inner_angle_des)*cos(-from_H_des);sin(inner_angle_des)*sin(-from_H_des)];
    dq_des = [dq(1);dq(2)]+2*l_tibia*[d_inner_angle_des*cos(inner_angle_des)*cos(-from_H_des) + d_from_H_des*sin(inner_angle_des)*sin(-from_H_des);
                                      d_inner_angle_des*cos(inner_angle_des)*sin(-from_H_des) - d_from_H_des*sin(inner_angle_des)*cos(-from_H_des)];
              
    A = D_gen(x);
    B = C_gen(x)*dq;
    G = G_gen(x);
    J = JSw_gen(x);  % jacobian for end effector
    %J = JCoM_gen(x);
    lambda = inv(J * inv(A) * J');
    Jbar = inv(A) * J' * lambda;
    mu = Jbar' * B - lambda * dJSw_gen(x) * dq;
    %mu = Jbar' * B - lambda * dJCoM_gen(x) * dq;
    p = Jbar' * G;
    kp = 500; kd = 500; % gains for this controller
    term = lambda * (-kp*(pSw - q_des) - kd*(dpSw -dq_des));
%     if (t > pi/2/t_con+0.1)
%         pSw-q_des
%         controlVersion = 6;
%         q_desired = q;
%     end
    F = mu + p + term;
    % null space term
    c_des = [0; 0.064286];
    dc_des = [0; 0];
    kp_c = 10; kd_c = 10;
    kp_c = 1000; kd_c = 100; % Jared testing
    N = eye(n) - Jbar*J;
    pC = robot_CoM_gen_2(x);
    dpC = robot_dCoM_gen(x);
    isolated_q = [q(1:5);q0(6:7)]; %jared testing
    isolated_dq = [dq(1:5);0;0]; % Jared Testing
    null_term = -kp_c*(q - q0(1:n)) -kd_c*(dq);%B + G + A*(-kd_c*(q - q0(1:n)));
    null_term = -kp_c*(isolated_q - q0(1:n)) -kd_c*(isolated_dq);%B + G + A*(-kd_c*(q - q0(1:n)));
    
    A = D_gen(x);
    B = C_gen(x)*dq;
    G = G_gen(x);
    JSt = JSt_gen(x);  % jacobian for end effector
    %J = JCoM_gen(x);
    lambda_St = inv(JSt * inv(A) * JSt');
    Jbar_St = inv(A) * JSt' * lambda_St;
    mu_St = Jbar_St' * B - lambda_St * dJSt_gen(x) * dq;
    %mu = Jbar' * B - lambda * dJCoM_gen(x) * dq;
    p_St = Jbar_St' * G;
    kp = 500; kd = 50; % gains for this controller
    q_des_St = pSt_gen(q0) + [-0.2;-.00];
    dq_des_St = [0;0];
    term_St = lambda_St * (-kp*(pSt - q_des_St) - kd*(dpSt -dq_des_St));
%     if (t > pi/2/t_con+0.1)
%         pSw-q_des
%         controlVersion = 6;
%         q_desired = q;
%     end
    FSt = mu_St + p_St + term_St;
%     null_term = zeros(7,1);

% ----------------------------------------------------
 kp_c = 1000; kd_c = 10; % Jared testing
    N2 = eye(n) - Jbar_St*JSt;
    pC = robot_CoM_gen_2(x);
    dpC = robot_dCoM_gen(x);
    isolated_q = [q(1:3);q0(4);q(5);q0(6:7)]; %jared testing
    isolated_dq = [dq(1:3);0;dq(5);0;0]; % Jared Testing
    null_term = -kp_c*(q - q0(1:n)) -kd_c*(dq);%B + G + A*(-kd_c*(q - q0(1:n)));
    null_term = -kp_c*(isolated_q - q0(1:n)) -kd_c*(isolated_dq);%B + G + A*(-kd_c*(q - q0(1:n)));
%     null_term = zeros(7,1);
    tau = J' * F + N'*null_term;
    %-------------------------------------------------------------------------


    tau = J' * F + N'*(JSt'*FSt+N2'*null_term);%null_term;
    u = tau(4:end);
%% Jared's but more curvy
case 8  %modification of 7.5
    %constants;
    n = NDoF_gen(x);
    q = x(1:n);
    dq = x(n+1:end);
    
    % Initial conditions
    pSw = pSw_gen(x);  % current position of end effector
    dpSw = dpSw_gen(x); % current velocity of end effector
    %pSw = robot_CoM_gen_2(x);
    %dpSw = robot_dCoM_gen(x);
    omega1 = 0;  %omega_hip
    omega2 = 0;  %omega_knee
    q0 = [C.x0; C.y0; C.q_torso0; C.q1_hip0-0.5; C.q1_knee0+.1; C.q2_hip0; C.q2_knee0;0;0;0;0;0;0;0];
    %x_des = q0 + [0;0;0;0;0; omega1*t; omega2*t;0;0;0;0;0;omega1; omega2];
    t_con = 8;
    x_mag = 1.2;
    y_mag = 0.7;
    t_con = 10; % Jared testing
    x_mag = 1.2; % Jared testing
    y_mag = 0.5; % Jared testing
    q_des = [x_mag*sin(t_con*t)*(t<pi/2/t_con) + x_mag*(t>=pi/2/t_con);y_mag*sin(t_con*t)*(t<pi/2/t_con) + y_mag*(t>=pi/2/t_con)];%.064286];%pSw_gen(x_des);  %q_des = desired position of the end effector
                             %dq_des = desired velocity of the end effector
    dq_des = [x_mag*t_con*cos(t_con*t)*(t<pi/2/t_con);y_mag*t_con*cos(t_con*t)*(t<pi/2/t_con)];%[omega1*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t) + (omega1+omega2)*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t-q2_knee0-omega2*t);...
              %-omega1*.5*cos(pi/2-q_torso0-q2_hip0-omega1*t) - (omega1+omega2)*.5*sin(pi/2-q_torso0-q2_hip0-omega1*t-q2_knee0-omega2*t)];
    
              
    inner_angle_0 = (pi-q0(7))/2;
    inner_angle = (pi-q(7))/2;
    from_H_0 = q0(6)-inner_angle_0;
    inner_angle_f = pi/6;
    global p_target
    if p_target(2)<0
        target_angle = 0;
    else
        target_angle = target_angle_gen(x);
    end
    t_tot_d = .3;
    
    inner_angle_des = inner_angle_0 + (inner_angle_f - inner_angle_0)/t_tot_d * min(t,t_tot_d);
    d_inner_angle_des = (inner_angle_f - inner_angle_0)/t_tot_d;
    if t > t_tot_d
        inner_angle_f = pi/2*.8;
        inner_angle_des = inner_angle_0 + (inner_angle_f - inner_angle_0)/t_tot_d * min(t-t_tot_d,t_tot_d);
        d_inner_angle_des = (inner_angle_f - inner_angle)/.03;
    end
    d_from_H_des = (-target_angle - from_H_0)/t_tot_d;
    if t > t_tot_d
        d_from_H_des = target_angle-kick_angle_gen(x);
    end
    from_H_des = from_H_0 + (-target_angle - from_H_0)/t_tot_d  * min(t,t_tot_d);
    
    q_des = [q(1);q(2)]+2*l_tibia*[sin(inner_angle_des)*cos(-from_H_des);sin(inner_angle_des)*sin(-from_H_des)];
    dq_des = [dq(1);dq(2)]+2*l_tibia*[d_inner_angle_des*cos(inner_angle_des)*cos(-from_H_des) + d_from_H_des*sin(inner_angle_des)*sin(-from_H_des);
                                      d_inner_angle_des*cos(inner_angle_des)*sin(-from_H_des) - d_from_H_des*sin(inner_angle_des)*cos(-from_H_des)];
              
    A = D_gen(x);
    B = C_gen(x)*dq;
    G = G_gen(x);
    J = JSw_gen(x);  % jacobian for end effector
    %J = JCoM_gen(x);
    lambda = inv(J * inv(A) * J');
    Jbar = inv(A) * J' * lambda;
    mu = Jbar' * B - lambda * dJSw_gen(x) * dq;
    %mu = Jbar' * B - lambda * dJCoM_gen(x) * dq;
    p = Jbar' * G;
    kp = 500; kd = 500; % gains for this controller
    term = lambda * (-kp*(pSw - q_des) - kd*(dpSw -dq_des));
%     if (t > pi/2/t_con+0.1)
%         pSw-q_des
%         controlVersion = 6;
%         q_desired = q;
%     end
    F = mu + p + term;
    % null space term
    c_des = [0; 0.064286];
    dc_des = [0; 0];
    kp_c = 10; kd_c = 10;
    kp_c = 1000; kd_c = 100; % Jared testing
    N = eye(n) - Jbar*J;
    pC = robot_CoM_gen_2(x);
    dpC = robot_dCoM_gen(x);
    isolated_q = [q(1:5);q0(6:7)]; %jared testing
    isolated_dq = [dq(1:5);0;0]; % Jared Testing
    null_term = -kp_c*(q - q0(1:n)) -kd_c*(dq);%B + G + A*(-kd_c*(q - q0(1:n)));
    null_term = -kp_c*(isolated_q - q0(1:n)) -kd_c*(isolated_dq);%B + G + A*(-kd_c*(q - q0(1:n)));
%     null_term = zeros(7,1);
    tau = J' * F + N'*null_term;
    u = tau(4:end);
    
%     Jtor = 

% Trying to get stance foot
%% Zero Dynamics
otherwise
u = [0; 0; 0; 0];

end

