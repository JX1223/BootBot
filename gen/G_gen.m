function G = G_gen(in1)
%G_GEN
%    G = G_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    21-Nov-2020 23:21:31

q1_hip = in1(4,:);
q2_hip = in1(6,:);
q1_knee = in1(5,:);
q2_knee = in1(7,:);
q_torso = in1(3,:);
t2 = q1_hip+q_torso;
t3 = q2_hip+q_torso;
t4 = q1_knee+t2;
t5 = q2_knee+t3;
t6 = sin(t2);
t7 = sin(t3);
t8 = sin(t4);
t9 = sin(t5);
t10 = t6.*1.839375e+1;
t11 = t7.*1.839375e+1;
t12 = -t10;
t13 = -t11;
t14 = t8.*(9.81e+2./1.6e+2);
t15 = t9.*(9.81e+2./1.6e+2);
t16 = -t14;
t17 = -t15;
G = [0.0;3.4335e+2;t12+t13+t16+t17-sin(q_torso).*(9.81e+2./4.0e+1);t12+t16;t16;t13+t17;t17];