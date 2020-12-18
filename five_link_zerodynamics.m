function dx = five_link_zerodynamics(t,x)
    n_DOF = NDoF_gen(x);
    u = [0;0;0;0];
    FSt_ = FSt_gen(x,u);
    dx = zeros(size(x));
    dx(1:n_DOF,1) = x(n_DOF+1:end,1);
    D_ = D_gen(x);
    C_ = C_gen(x);
    G_ = G_gen(x);
    B_ = B_gen(x);
    JSt_ = JSt_gen(x);
    dx(n_DOF+1:end,1) = D_ \ (JSt_'*FSt_ + B_*u - C_*x(n_DOF+1:end,1) - G_);
end