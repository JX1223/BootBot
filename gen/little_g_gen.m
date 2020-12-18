function g = little_g_gen(s, tau)

D = D_gen(s);
B = B_gen(s);
G = G_gen(s);
dq = dq_gen(s);
JSt = JSt_gen(s);
FSt_u = FSt_u_gen(s,tau);
g = [zeros(5,2); D\(JSt'*FSt_u+B)];

end