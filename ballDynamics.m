function ds = ballDynamics(t,state)
    D = D_gen_ball(state);
    C = C_gen_ball(state);
    B = B_gen_ball(state);
    G = G_gen_ball(state);
    
    % Jason - I'm not sure what this does
    if  t>2.5 && t<3
        Fext = [0;0];%Fext = [500;0]; % Eric made this change
    else
        Fext = [0;0];
    end
    
    % Jcon appears to be constant
    Jcon = [1, 0, 0;
            0, 1, 0];

    % Evaluate dynamics
    dq = state(4:6);
    d2q = D\(-C*dq-G +Jcon'*Fext);
    ds = [dq;d2q];
end


