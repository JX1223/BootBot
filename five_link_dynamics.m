function dx = five_link_dynamics(t,x)
    global Fext
    Jext = JpComTorso_gen(x);
    
    FSw = [0;0];
    JSw = JSw_gen(x);
    JFext = Jext'*Fext+JSw'*FSw;


    n_DOF = NDoF_gen(x);
    u = controller(t,x);
    FSt_ = FSt_gen(x,u,JFext);
    dx = zeros(size(x));
    dx(1:n_DOF,1) = x(n_DOF+1:end,1);
    D_ = D_gen(x);
    C_ = C_gen(x);
    G_ = G_gen(x);
    B_ = B_gen(x);
    JSt_ = JSt_gen(x);
    dx(n_DOF+1:end,1) = D_ \ (JSt_'*FSt_ + B_*u - C_*x(n_DOF+1:end,1) - G_ + JFext);
end