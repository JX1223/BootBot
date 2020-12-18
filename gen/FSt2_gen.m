% FSt = - pinv(JSt(D\JSt'))(JSt(D(-H + Btau)) + dJStdq + 2alphaJStdq + alpha^2*pst);
function FSt = FSt2_gen(s,tau,JFext)
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
    dpst = dpSt2_gen(s);

    
    FSt = - pinv(JSt*(D\JSt'))*(JSt*(D\(-H + B*tau + JFext)) + dJSt*dq + 2*alpha*JSt*dq + alpha^2*pSt);
    mu_s = 0.8;
    
    
    threshold = .01;
    if FSt(2) < 0  || pSt(2) > threshold 
        FSt(2) = 0;
    end
    if abs(FSt(1)) > mu_s*FSt(2)
        FSt(1) = sign(FSt(1))*mu_s*FSt(2);
    end
    if FSt(4) < 0  || pSt(4) > threshold 
        FSt(4) = 0;
    end
    if abs(FSt(3)) > mu_s*FSt(4)
        FSt(3) = sign(FSt(3))*mu_s*FSt(4);
    end
    k_floor = 100000;
    c_floor = 10000;
    if pSt(2)<0
        FSt(2) = FSt(2) - k_floor*pSt(2)-c_floor*dpst(2);
    end
    if pSt(4)<0
        FSt(4) = FSt(4) - k_floor*pSt(4)-c_floor*dpst(4);
    end
    k_gain = 1;
    if abs(dpst(1)) > 0.01 % was .1 beforme messing around
        sampleFSt = mu_s*FSt(2);
        if FSt(1)/sign(dpst(1)) < 0 && abs(FSt(1)) > abs(sampleFSt)
            
        else
            FSt(1) = max(-400,min(-sign(dpst(1))*mu_s*FSt(2)*k_gain,400));
        end
            
        
    end
    if abs(dpst(3)) > 0.1
        sampleFSt = mu_s*FSt(4);
        if FSt(3)/sign(dpst(3)) < 0 && abs(FSt(3)) > abs(sampleFSt)
            
        else
            FSt(3) = max(-600,min(-sign(dpst(3))*mu_s*FSt(4),600));
        end
            
    end
end