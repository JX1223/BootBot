function f = f_gen(s, tau)

D = D_gen(s);
C = C_gen(s);
G = G_gen(s);
dq = dq_gen(s);
JSt = JSt_gen(s);
FSt_nu = FSt_nu_gen(s,tau);
f = [s(6:10); D\(JSt'*FSt_nu - C*dq -G)];

end