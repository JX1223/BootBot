function utot = tauTotal(f_c,x)
if length(f_c) ==2
    tau = JSt_gen(x)'*f_c;
else
    tau = JSt2_gen(x)'*f_c;
end
    u = tau(4:end);
    utot = u'*u;
end

