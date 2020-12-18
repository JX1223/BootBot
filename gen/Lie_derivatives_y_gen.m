function [Lfy, Lgy, Lf2y, LgLfy] = Lie_derivatives_y_gen(s, tau, y, JFext)
syms x y q_torso q1_hip q1_knee q2_hip q2_knee
syms dx dy dq_torso dq1_hip dq1_knee dq2_hip dq2_knee
syms u1 u2 u3 u4
tau = [u1;u2;u3;u4];
f = f_gen(s, tau, JFext);
g = little_g_gen(s,tau);
Lfy = jacobian(y, s)*f_s;
Lgy = jacobian(y, s)*g_s;
Lf2y = jacobian(Lfy, s)*f_s;
LgLfy = jacobian(Lfy,s)*g_s;

end