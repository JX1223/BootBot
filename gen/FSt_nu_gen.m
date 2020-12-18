function FSt_nu = FSt_nu_gen(s,tau)
    FSt = FSt_gen(s);
    FSt_u = jacobian(FSt, tau);
    FSt_nu = (FSt - FSt_u*tau);
end