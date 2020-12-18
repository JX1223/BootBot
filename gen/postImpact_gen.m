function [postImpact] = postImpact_gen(s)
    D = D_gen(s);
    JSw = JSw_gen(s);
    dq = dq_gen(s);
    [postImpact] = ([D, -JSw';JSw, zeros(2)])\[D*dq;zeros(2,1)];

end