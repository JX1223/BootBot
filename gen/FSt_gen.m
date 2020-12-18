% FSt = - pinv(JSt(D\JSt'))(JSt(D(-H + Btau)) + dJStdq + 2alphaJStdq + alpha^2*pst);
function FSt = FSt_gen(s,tau,JFext)
    mu_s = 0.8;
    pSt = pSt_gen(s);

    D = D_gen(s);
    B = B_gen(s);
    C = C_gen(s);
    G = G_gen(s);
    JSt = JSt_gen(s);
    dJSt = dJSt_gen(s);
    alpha = 0;
    H = H_gen(s);
    dq = dq_gen(s);
    pst = pSt_gen(s);
    dpst = dpSt_gen(s);
    FSt = - pinv(JSt*(D\JSt'))*(JSt*(D\(-H + B*tau + JFext)) + dJSt*dq + 2*alpha*JSt*dq + alpha^2*pst);

    threshold = .01;
    k_floor = 100000;

    if FSt(2) < 0  || pSt(2) > threshold 
        FSt(2) = 0;
    end
    if abs(FSt(1)) > mu_s*FSt(2)
        FSt(1) = sign(FSt(1))*mu_s*FSt(2);
    end
    if pSt(2)<0
        FSt(2) = FSt(2) - k_floor*pSt(2);
    end
    
    if abs(dpst(1)) > 0.01
        FSt(1) = -sign(dpst(1))*(mu_s)*FSt(2);
    end