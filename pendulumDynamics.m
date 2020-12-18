function dx = pendulumDynamics(t,state)
syms theta
syms dtheta


q = theta;
dq = dtheta;
s = [q;dq];

% Tether location
x = 3;
y = 1.8;

g = 9.81;

l = 1;
m_ball = 10;
rad = .1;

theta0 = deg2rad(30);

pCoM = [x + l*sin(theta); y-l*cos(theta)];
matlabFunction(pCoM, 'File', 'gen/pen_CoM_gen', 'Vars', {s});

dpCoM = jacobian(pCoM, q)*dq;

KE = .5*m_ball*(dpCoM(1)^2+dpCoM(2)^2);

PE = m_ball*g*pCoM(2);

L = KE - PE;

% Equations of Motion
EOM = jacobian(jacobian(L,dq), q)*dq - jacobian(L, q)' ;
EOM = simplify(EOM);

% Find the D, C, G, and B matrices

% Actuated variables
qActuated = [];

% D, C, G, and B matrices
[D, C, G, B] = LagrangianDynamics(KE, PE, q, dq, qActuated); % LONG
% matlabFunction(D, 'File', 'pengen/D_gen', 'Vars', {s});
% matlabFunction(C, 'File', 'pengen/C_gen', 'Vars', {s});
% matlabFunction(G, 'File', 'pengen/G_gen', 'Vars', {s});
% matlabFunction(B, 'File', 'pengen/B_gen', 'Vars', {s});

if abs(state(1)) <deg2rad(30) && t>2.5 && t<3
    Fext = [1000;0];
else
    Fext = [0;0];
end
    
pcon = pCoM - [rad;0];
Jcon = jacobian(pcon, q);

d2q = D\(-C*dq-G +Jcon'*Fext);
dx = [dq;d2q];


theta = state(1);
dtheta = state(2);

dx = eval(dx);
end


