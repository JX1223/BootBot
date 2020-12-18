% FSt = - pinv(JSt(D\JSt'))(JSt(D(-H + Btau)) + dJStdq + 2alphaJStdq + alpha^2*pst);
function FSt = FSt2Cart_gen(s,tau,Fext,Jext)
    JFext = Jext'*Fext;
    D = D_gen(s);
    B = B_gen(s);
    C = C_gen(s);
    G = G_gen(s);
    JSt = JSt2_gen(s);
    dJSt = dJSt2_gen(s);
    alpha = 0;
    H = H_gen(s);
    dq = dq_gen(s);
    pSt = pSt2_gen(s);
    
    FSt = - pinv(JSt*(D\JSt'))*(JSt*(D\(-H + B*tau + JFext)) + dJSt*dq + 2*alpha*JSt*dq + alpha^2*pSt);
    mu_s = 0.8;
    
    
    threshold = .01;
    if FSt(2) < 0  || pSt(2) > threshold 
        FSt(2) = 0;
    end
    if abs(FSt(1)) > mu_s*FSt(2)
        FSt(1) = sign(FSt(1))*mu_s*FSt(2);
    end
    k_floor = 100000;
    if pSt(2)<0
        FSt(2) = FSt(2) - k_floor*pSt(2);
    end
    if pSt(4)<0
        FSt(4) = FSt(4) - k_floor*pSt(4);
    end

    if Fext(1) ==0
        FSt(4) = 0;
    else
        a = 1;
    end
    FSt(3) = 0;
end