function ds = cartDynamics(t,state)
syms x l
syms dx dl

k = 1;
mcart = 10;
mtarget = 1;
l0 = .5;

q = [x;l];
dq = [dx; dl];
s = [q;dq];

p1 = x;
p2 = x-l;

dp1 = dx;
dp2 = dx - dl;

KE = .5*mcart*dp1^2 + .5*mtarget*dp2^2;

PE = .5*k*(l-l0)^2;

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

Fext = -k*(state(2)-l0);

    
pcon = p2;
Jcon = jacobian(pcon, q);

d2q = D\(-C*dq-G +Jcon'*Fext);
ds = [dq;d2q];


x = state(1);
l = state(2);
dx = state(3);
dl = state(4);

ds = eval(ds);
end



