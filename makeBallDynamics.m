% Obtain ball parameters (ball_radius, m_ball, J_ball)
constants; 
syms x y theta
syms dx dy dtheta

q = [x; y; theta];
dq = [dx;dy;dtheta];
s = [q;dq];

pCoM = [x ; y];
matlabFunction(pCoM, 'File', 'gen/ball_CoM_gen', 'Vars', {s});

dpCoM = jacobian(pCoM, q)*dq;

KE = .5*m_ball*(dpCoM(1)^2+dpCoM(2)^2) + .5*J_ball*dtheta^2;

PE = m_ball*g*pCoM(2);

L = KE - PE;

% % Equations of Motion
% EOM = jacobian(jacobian(L,dq), q)*dq - jacobian(L, q)' ;
% EOM = simplify(EOM);

% Find the D, C, G, and B matrices
% Actuated variables
qActuated = [];

% D, C, G, and B matrices
[D, C, G, B] = LagrangianDynamics(KE, PE, q, dq, qActuated);

% Save ball lagrangian dynamics as matlab function files
matlabFunction(D, 'File', 'gen/D_gen_ball', 'Vars', {s});
matlabFunction(C, 'File', 'gen/C_gen_ball', 'Vars', {s});
matlabFunction(G, 'File', 'gen/G_gen_ball', 'Vars', {s});
matlabFunction(B, 'File', 'gen/B_gen_ball', 'Vars', {s});

pcon = pCoM - [ball_radius/sqrt(2);ball_radius/sqrt(2)];
Jcon = jacobian(pcon, q);


