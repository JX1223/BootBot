function ds = ballDynamics_externalForce(t,state,Fext)
    % Assumes no spin
%     constants;
g = 9.81;
m_ball = 3;
ball_radius = 0.03*2;
    y = state(2);
    k_spring = 1000000;
    if y<ball_radius
        springF = -k_spring * (y-ball_radius);
    else
        springF = 0;
    end
    dq = state(4:5);
    d2q = Fext/m_ball+[0;-g+springF/m_ball];
    ds = [dq;0;d2q;0];
end


