function FSt_u = FSt_u_gen(s,tau)
    FSt = FSt_gen(s);
    FSt_u = jacobian(FSt, tau);
end