% Ayush Agrawal
% ayush.agrawal@berkeley.edu
% ME193B/293B Feedback Control of Legged Robots
% UC Berkeley

clear
%% Define symbolic variables for cofiguration variables and mechanical parameters

syms q1_hip q1_knee q2_knee q2_hip q_torso x y real
syms dq1_hip dq1_knee dq2_knee dq2_hip dq_torso dx dy real
syms u1_hip u1_knee u2_hip u2_knee real
syms contact_dist

% q1
% q2
% qB - Body angle
% 


% Position Variable vector
q = [x; y; q_torso; q1_hip; q1_knee; q2_hip; q2_knee];


% Velocity variable vector
dq = [dx; dy; dq_torso; dq1_hip; dq1_knee; dq2_hip; dq2_knee];



% State vector
s = [q;dq];
% matlabFunction(q, 'File', 'gen/q_gen', 'Vars', {s});
% matlabFunction(dq, 'File', 'gen/dq_gen', 'Vars', {s});

% Inputs
tau = [u1_hip;u1_knee;u2_hip;u2_knee];

% parameters
          
l_femur = 0.5; % Upper Leg
l_tibia = 0.5; % Lower Leg
l_torso = 0.5; % Torso
m_femur = 5/2;
m_tibia = 5/2;
m_torso = 10;
J_femur = 0;
J_tibia = 0;
J_torso = 0;
m_hip = 15;
g = 9.81;

% # of Degrees of freedom
NDof = length(q);


%% Problem 1: Lagrangian Dynamics
%Find the CoM position of each link

% Torso
pComTorso = [x + l_torso/2*sin(q_torso);...
             y + l_torso/2*cos(q_torso)];

% Leg 1 femur 
pKnee1 = [x + l_femur*cos( q1_hip + q_torso - deg2rad(90));...
           y - l_femur*sin( q1_hip + q_torso - deg2rad(90))];

% Leg 1 tibia 
pFoot1 = [pKnee1(1) + l_tibia*cos(q1_knee + q1_hip + q_torso - deg2rad(90));...
           pKnee1(2) - l_tibia*sin(q1_knee + q1_hip + q_torso - deg2rad(90))];
       
             
% Leg 2 femur 
pKnee2 = [x + l_femur*cos( q2_hip + q_torso - deg2rad(90));...
           y - l_femur*sin( q2_hip + q_torso - deg2rad(90))];

% Leg 2 tibia 
pFoot2 = [pKnee2(1) + l_tibia*cos(q2_knee + q2_hip + q_torso - deg2rad(90));...
           pKnee2(2) - l_tibia*sin(q2_knee + q2_hip + q_torso - deg2rad(90))];
             
% Leg 2 tibia 
pLeg2contact = [pKnee2(1) + contact_dist*cos(q2_knee + q2_hip + q_torso - deg2rad(90));...
           pKnee2(2) - contact_dist*sin(q2_knee + q2_hip + q_torso - deg2rad(90))];         
Jcontact = jacobian(pLeg2contact, q);
matlabFunction(Jcontact, 'File', 'gen/Jcontact_gen', 'Vars', {s,contact_dist});
       
       
% Leg 1 femur CoM
pComFemur1 = [x + l_femur/2*cos( q1_hip + q_torso - deg2rad(90));...
                 y - l_femur/2*sin( q1_hip + q_torso - deg2rad(90))];

% Leg 1 tibia CoM
pComTibia1 = [pKnee1(1) + l_tibia/2*cos(q1_knee + q1_hip + q_torso - deg2rad(90));...
                 pKnee1(2) - l_tibia/2*sin(q1_knee + q1_hip + q_torso - deg2rad(90))];
             
% Leg 2 femur CoM
pComFemur2 = [x + l_femur/2*cos( q2_hip + q_torso - deg2rad(90));...
                 y - l_femur/2*sin( q2_hip + q_torso - deg2rad(90))];

% Leg 2 tibia CoM
pComTibia2 = [pKnee2(1) + l_tibia/2*cos(q2_knee + q2_hip + q_torso - deg2rad(90));...
                 pKnee2(2) - l_tibia/2*sin(q2_knee + q2_hip + q_torso - deg2rad(90))];

% Finding position, jacobian, and derivative of jacobian of CoM
totalMass = 2*m_femur+2*m_tibia+m_torso+m_hip;
pmHip = m_hip* [x;y];
CoM = (pComFemur1+ pComTibia1+pComFemur2+pComTibia2+pComTorso+pmHip)/totalMass;
matlabFunction(CoM, 'File', 'gen/robot_CoM_gen_2', 'Vars', {s});
JCoM = jacobian(CoM, q);
matlabFunction(JCoM, 'File', 'gen/JCoM_gen', 'Vars', {s});
% Compute the time derivative of the Jacobian
dJCoM = sym(zeros(size(JCoM)));
for i = 1:size(JCoM, 1)
    for j = 1:size(JCoM, 2)
        dJCoM(i, j) = simplify(jacobian(JCoM(i, j), q)*dq);
    end
end
matlabFunction(dJCoM, 'File', 'gen/dJCoM_gen', 'Vars', {s});

% Find the CoM velocity of each link

% Torso
dpComTorso = simplify(jacobian(pComTorso, q)*dq);

% Leg 1
dpComFemur1 = simplify(jacobian(pComFemur1, q)*dq);
dpComTibia1 = simplify(jacobian(pComTibia1, q)*dq);

% Leg 2
dpComFemur2 = simplify(jacobian(pComFemur2, q)*dq);
dpComTibia2 = simplify(jacobian(pComTibia2, q)*dq);

totalMass = 2*m_femur+2*m_tibia+m_torso+m_hip;
dpmF1 = m_femur * dpComFemur1;
dpmT1 = m_tibia * dpComTibia1;
dpmF2 = m_femur * dpComFemur2;
dpmT2 = m_tibia * dpComTibia2;
dpmTor = m_torso * dpComTorso;
dpmHip = m_hip* [dx;dy];

dCoM = (dpmF1+ dpmT1+dpmF2+dpmT2+dpmTor+dpmHip)/totalMass;
matlabFunction(dCoM, 'File', 'gen/robot_dCoM_gen', 'Vars', {s});
%% Find absolute angular velocity associated with each link:
% Torso
dqTorsoAbsolute = dq_torso;
% Leg 1
dq1FemurAbsolute = dq_torso + dq1_hip;
dq1TibiaAbsolute = dq_torso + dq1_hip + dq1_knee;
% Leg 2
dq2FemurAbsolute = dq_torso + dq2_hip;
dq2TibiaAbsolute = dq_torso + dq2_hip + dq2_knee;

% Total Kinetic energy = Sum of kinetic energy of each link

% Torso
KETorso = 0.5*m_torso*dpComTorso(1)^2 + 0.5*m_torso*dpComTorso(2)^2 + 0.5*J_torso*dqTorsoAbsolute^2;

% Leg 1
KEFemur1 = 0.5*m_femur*dpComFemur1(1)^2 + 0.5*m_femur*dpComFemur1(2)^2 + 0.5*J_femur*dq1FemurAbsolute^2;
KETibia1 = 0.5*m_tibia*dpComTibia1(1)^2 + 0.5*m_tibia*dpComTibia1(2)^2 + 0.5*J_tibia*dq1TibiaAbsolute^2;

% Leg 2
KEFemur2 = 0.5*m_femur*dpComFemur2(1)^2 + 0.5*m_femur*dpComFemur2(2)^2 + 0.5*J_femur*dq2FemurAbsolute^2;
KETibia2 = 0.5*m_tibia*dpComTibia2(1)^2 + 0.5*m_tibia*dpComTibia2(2)^2 + 0.5*J_tibia*dq2TibiaAbsolute^2;

KEHip = 0.5*m_hip*dx^2 + 0.5*m_hip*dy^2;
% Total KE
KE = simplify(KETorso + KETibia1 + KEFemur1 + KETibia2 + KEFemur2 +KEHip);

% Total potential energy = Sum of Potential energy of each link

% Torso
PETorso = m_torso*g*pComTorso(2);

%Leg 1
PETibia1 = m_tibia*g*pComTibia1(2);
PEFemur1 = m_femur*g*pComFemur1(2);


% Leg 2
PETibia2 = m_tibia*g*pComTibia2(2);
PEFemur2 = m_femur*g*pComFemur2(2);

% Hip
PEHip = m_hip*g*y;

% Total PE
PE = simplify(PETorso + PETibia1 + PEFemur1 + PETibia2 + PEFemur2 + PEHip);

% Lagrangian

L = KE - PE;

% Equations of Motion
EOM = jacobian(jacobian(L,dq), q)*dq - jacobian(L, q)' ;
EOM = simplify(EOM);

% Find the D, C, G, and B matrices

% Actuated variables
qActuated = [q1_hip;q1_knee;q2_hip;q2_knee];

% D, C, G, and B matrices
[D, C, G, B] = LagrangianDynamics(KE, PE, q, dq, qActuated); % LONG
% matlabFunction(D, 'File', 'gen/D_gen', 'Vars', {s});
% matlabFunction(C, 'File', 'gen/C_gen', 'Vars', {s});
% matlabFunction(G, 'File', 'gen/G_gen', 'Vars', {s});
% matlabFunction(B, 'File', 'gen/B_gen', 'Vars', {s});

%%  Dynamics of Systems with Constraints
%Compute the Ground reaction Forces

% Compute the position of the stance foot (Leg 1) 
pSt = pFoot1;
pSt2 = [pFoot1;pFoot2];
matlabFunction(pSt, 'File', 'gen/pSt_gen', 'Vars', {s});
matlabFunction(pSt2, 'File', 'gen/pSt2_gen', 'Vars', {s});
dpSt = simplify(jacobian(pSt, q)*dq);
dpSt2 = simplify(jacobian(pSt2, q)*dq);
matlabFunction(dpSt, 'File', 'gen/dpSt_gen', 'Vars', {s});
matlabFunction(dpSt2, 'File', 'gen/dpSt2_gen', 'Vars', {s});

JpComTorso = jacobian(pComTorso, q);
matlabFunction(JpComTorso, 'File', 'gen/JpComTorso_gen', 'Vars', {s});

% Compute the jacobian of the stance foot
JSt = jacobian(pSt, q);
matlabFunction(JSt, 'File', 'gen/JSt_gen', 'Vars', {s});
JSt2 = jacobian(pSt2, q);
matlabFunction(JSt2, 'File', 'gen/JSt2_gen', 'Vars', {s});

% Compute the time derivative of the Jacobian
dJSt = sym(zeros(size(JSt)));
for i = 1:size(JSt, 1)
    for j = 1:size(JSt, 2)
        dJSt(i, j) = simplify(jacobian(JSt(i, j), q)*dq);
    end
end
matlabFunction(dJSt, 'File', 'gen/dJSt_gen', 'Vars', {s});

dJSt2 = sym(zeros(size(JSt2)));
for i = 1:size(JSt2, 1)
    for j = 1:size(JSt2, 2)
        dJSt2(i, j) = simplify(jacobian(JSt2(i, j), q)*dq);
    end
end
matlabFunction(dJSt2, 'File', 'gen/dJSt2_gen', 'Vars', {s});

% Compute the time derivative of the Jacobian
dJpComTorso = sym(zeros(size(JpComTorso)));
for i = 1:size(JpComTorso, 1)
    for j = 1:size(JpComTorso, 2)
        dJpComTorso(i, j) = simplify(jacobian(JpComTorso(i, j), q)*dq);
    end
end
matlabFunction(dJpComTorso, 'File', 'gen/dJpComTorso_gen', 'Vars', {s});

H = C*dq + G;
matlabFunction(H, 'File', 'gen/H_gen', 'Vars', {s});

alpha = 0;
% Constraint Force to enforce the holonomic constraint:
% Dinv = inv(D);
% zerothpart = simplify(JSt*(Dinv*JSt'));
% zerothpart_linv_1 = zerothpart'*zerothpart;
% zerothpart_linv_2 = simplify(zerothpart_linv_1);
% zerothpart_linv_3 = inv(zerothpart_linv_2);
% Zpinv = zerothpart_linv_2 * zerothpart';
% firstpart = simplify(- pinv(zerothpart));
firstpart = simplify(- Zpinv);
secondpart = simplify((JSt*(Dinv*(-H + B*tau))));
thirdpart = simplify(dJSt*dq);
fourthpart = simplify(2*alpha*JSt*dq);
fifthpart = simplify(alpha^2*pSt);
FSt = firstpart*(secondpart + thirdpart + fourthpart + fifthpart);
FSt = simplify(FSt);

% Split FSt into 2 components: 1. which depends on tau and 2. which does
% not depend on tau 
% Note: FSt is linear in tau

FSt_u = jacobian(FSt, tau); % FSt = Fst_u*tau + (Fst - Fst_u*tau)
% Fst_nu = simplify(FSt - Fst_u*tau); % Fst_nu = (Fst - Fst_u*tau)
FSt_nu = (FSt - FSt_u*tau);

%% Impact Map

% Compute the swing leg position (leg 2)
pSw = pFoot2;
matlabFunction(pSw, 'File', 'gen/pSw_gen', 'Vars', {s});


JSw = jacobian(pSw, q);
matlabFunction(JSw, 'File', 'gen/JSw_gen', 'Vars', {s});

dJSw = sym(zeros(size(JSw)));
for i = 1:size(JSw, 1)
    for j = 1:size(JSw, 2)
        dJSw(i, j) = simplify(jacobian(JSw(i, j), q)*dq);
    end
end
matlabFunction(dJSw, 'File', 'gen/dJSw_gen', 'Vars', {s});

% postImpact = [qPlus;F_impact];
% Here, q, dq represent the pre-impact positions and velocities
[postImpact] = ([D, -JSw';JSw, zeros(2)])\[D*dq;zeros(2,1)];

% Post Impact velocities
dqPlus = simplify(postImpact(1:NDof));

% Impact Force Magnitude
Fimpact = simplify(postImpact(NDof+1:NDof+2));


%% Other functions

% swing foot velocity
dpSw = JSw*dq;
matlabFunction(dpSw, 'File', 'gen/dpSw_gen', 'Vars', {s});

%% Export functions
if ~exist('./gen')
    mkdir('./gen')
end
addpath('./gen')

matlabFunction(FSt, 'File', 'gen/Fst_gen', 'Vars', {s, tau});
matlabFunction(dqPlus, 'File', 'gen/dqPlus_gen', 'Vars', {s});
matlabFunction(pSw, 'File', 'gen/pSw_gen', 'Vars', {s});
matlabFunction(dpSw, 'File', 'gen/dpSw_gen', 'Vars', {s});
matlabFunction(pSt, 'File', 'gen/pSt_gen', 'Vars', {s});
matlabFunction(pComTibia1, 'File', 'gen/pComTibia1_gen', 'Vars', {s});
matlabFunction(pComFemur1, 'File', 'gen/pComFemur1_gen', 'Vars', {s});
matlabFunction(pComTibia2, 'File', 'gen/pComTibia2_gen', 'Vars', {s});
matlabFunction(pComFemur2, 'File', 'gen/pComFemur2_gen', 'Vars', {s});
matlabFunction(pComTorso, 'File', 'gen/pComTorso_gen', 'Vars', {s});
matlabFunction(pKnee1, 'File', 'gen/pKnee1_gen', 'Vars', {s});
matlabFunction(pFoot1, 'File', 'gen/pFoot1_gen', 'Vars', {s});
matlabFunction(pKnee2, 'File', 'gen/pKnee2_gen', 'Vars', {s});
matlabFunction(pFoot2, 'File', 'gen/pFoot2_gen', 'Vars', {s});
matlabFunction(dpComTorso, 'File', 'gen/dpComTorso_gen', 'Vars', {s});




%% [Part 1a] Compute the f and g vectors
% D * d2q + C*dq + G = B*u + JSt'*FSt
% d2q = inv(D)*(B*u+JSt'*FSt - Cdq -G)
%     = inv(D)*(JSt'*Fst_nu - Cdq -G) + inv(D)*(B*u+JSt'*Fst_u*u)
% ds = [dq; d2q] = [s(2), d2q]
struct5 = load('HW5_initialize.mat');
g = simplify([zeros(5,2); D\(JSt'*FSt_u+B)]);
f = simplify([s(6:10); D\(JSt'*FSt_nu - C*dq -G)]);
disp(0)
matlabFunction(f, 'File', 'gen/auto_f', 'Vars', {s});
disp(1)
matlabFunction(g, 'File', 'gen/auto_g', 'Vars', {s});
disp(2)

% ds = f + g*[u1;u2];
% matlabFunction(ds, 'File', 'auto_ds') ;
% matlabFunction(ds, 'File', 'gen/auto_ds', 'Vars', {s, u1, u2});



%% Change of Coordinates
% Transformation matrix:
T = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 1;
     0 0 0 1 1;
     0 0 0 0 1];
 
 T = [1 0 0 0 0 0 0 ;
      0 1 0 0 0 0 0 ; 
      0 0 1 0 0 0 0 ;
      0 0 1 1 0 0 0 ;
      0 0 1 1 1 0 0 ;
      0 0 1 0 0 1 0 ;
      0 0 1 0 0 1 1 ];
 
d = [0;
     0;
     0;
     -pi;
     -pi;
     -pi;
     -pi];

%% [Part 1b] Output dynamics
% Fill up this
syms a1 a2 a3 a0
syms b0 b1 b2 b3
alpha=[a0; a1; a2; a3];
beta = [b0; b1; b2; b3];
abs_angs = T*q+d;
theta_1 = abs_angs(3);

mid = beta(1) + beta(2)*theta_1 + beta(3)*theta_1^2 + beta(4)*theta_1^3;
theta_1_d = pi/8;
h2 = -(theta_1)+mid*(theta_1+theta_1_d)*(theta_1-theta_1_d);
    
h1 = a0 + a1*theta_1 + a2*theta_1^2 + a3*theta_1^3;
y = [abs_angs(5) - h1; abs_angs(4) - h2];
matlabFunction(y, 'File', 'gen/auto_y', 'Vars', {s, alpha, beta});
dy = jacobian(y,q)*dq;
matlabFunction(dy, 'File', 'gen/auto_dy', 'Vars', {s, alpha, beta});


%% [Part 1c] Lie Derivatives
dy_ds = jacobian(y, s);
Lf_y = simplify(dy_ds*f)
matlabFunction(Lf_y, 'File', 'gen/auto_Lf_y', 'Vars', {s, alpha, beta});

Lg_y = simplify(dy_ds*g)
matlabFunction(Lg_y, 'File', 'gen/auto_Lg_y', 'Vars', {s, alpha, beta});

L2_ = jacobian(Lf_y, s);
L2f_y = simplify(L2_ * f)
matlabFunction(L2f_y, 'File', 'gen/auto_L2f_y', 'Vars', {s, alpha, beta});
Lgf_y = simplify(L2_ * g)
matlabFunction(Lgf_y, 'File', 'gen/auto_Lgf_y', 'Vars', {s, alpha, beta});






%% [Part 1d] Relabelling Matrix
% Fill up this

% R = [1 0 0 0 0 ;
%      0 1 0 0 0 ; 
%      0 0 0 1 0 ;
%      0 0 1 0 0 ;
%      0 0 0 0 1 ];
 R = [1 0 0 0 0 0 0 ;
      0 1 0 0 0 0 0 ;
      0 0 1 0 0 0 0 ;
      0 0 0 0 0 1 0 ;
      0 0 0 0 0 0 1 ;
      0 0 0 1 0 0 0 ;
      0 0 0 0 1 0 0 ];
 
AA = [  D     -JSw';
       JSw   zeros(2)  ];
bb = [D*dq; 0; 0 ];
delta = AA\bb;

relabel_q = R*q;
relabel_dq = R*delta(1:5);

matlabFunction(relabel_q, 'File', 'gen/relabel_q_gen', 'Vars', {s});
matlabFunction(relabel_dq, 'File', 'gen/auto_relabel_dq', 'Vars', {s});

