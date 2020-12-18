function dx = five_link_dynamics_2(t,x,C)
    global pos_est vel_est ballForecastTime ballForecast lidarSwitch
    s_target = interp1(ballForecastTime , ballForecast,t);
    if (lidarSwitch)
        [pos_est, vel_est] = ballEstimator(t,x, s_target', C);
    end
    n_DOF = NDoF_gen(x);
    u = controller(t,x,C);
    global Fext
    Jext = JpComTorso_gen(x);
    JFext = Jext'*Fext;
    FSt_ = FSt2_gen(x,u,JFext);
    JSt_ = JSt2_gen(x);
    dx = zeros(size(x));
    dx(1:n_DOF,1) = x(n_DOF+1:end,1);
    D_ = D_gen(x);
    C_ = C_gen(x);
    G_ = G_gen(x);
    B_ = B_gen(x);

    dx(n_DOF+1:end,1) = D_ \ (JSt_'*FSt_ + B_*u - C_*x(n_DOF+1:end,1) - G_ + JFext);
end