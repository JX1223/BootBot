function Fext = cartDynamics_masslessTarget_forceCalculator(t,s,tau)

% State of robot
s_robot = s(1:14);

% State of target
s_target = s(15:end);
x_target = s_target(1);

pSw = pSw_gen(s_robot);
x_foot = pSw(1);

springLength = 1;
k = 10000;

dist = x_target - x_foot;

Fext = [0;0];

if dist < springLength
    mu_t = 0;
    k_d = 1000;
    Fext(1) = k*(springLength-dist);
    dpSw = dpSw_gen(s_robot);
    if abs(dpSw(2))>.1
    Fext(2) = sign(dpSw(2))*mu_t*Fext(1);
    Fext(2) = k_d *dpSw(2);
    Fext(2) = max(-mu_t*Fext(1), min(mu_t*Fext(1),Fext(2)));
    end
    % + JSt'*Fex 
end
    

end



