function H = H_gen(in1)
%H_GEN
%    H = H_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    28-Nov-2020 00:06:19

dq1_hip = in1(11,:);
dq2_hip = in1(13,:);
dq1_knee = in1(12,:);
dq2_knee = in1(14,:);
dq_torso = in1(10,:);
q1_hip = in1(4,:);
q2_hip = in1(6,:);
q1_knee = in1(5,:);
q2_knee = in1(7,:);
q_torso = in1(3,:);
t2 = sin(q1_knee);
t3 = sin(q2_knee);
t4 = sin(q_torso);
t5 = dq1_hip+dq_torso;
t6 = dq2_hip+dq_torso;
t7 = q1_hip+q_torso;
t8 = q2_hip+q_torso;
t9 = dq1_knee+t5;
t10 = dq2_knee+t6;
t11 = cos(t7);
t12 = cos(t8);
t13 = q1_knee+t7;
t14 = q2_knee+t8;
t15 = sin(t7);
t16 = sin(t8);
t21 = dq1_hip.*dq1_knee.*t2.*(5.0./1.6e+1);
t22 = dq2_hip.*dq2_knee.*t3.*(5.0./1.6e+1);
t17 = cos(t13);
t18 = cos(t14);
t19 = sin(t13);
t20 = sin(t14);
t23 = t11.*(1.5e+1./8.0);
t24 = t12.*(1.5e+1./8.0);
t25 = -t21;
t26 = -t22;
t27 = t15.*(1.5e+1./8.0);
t28 = t16.*(1.5e+1./8.0);
t37 = dq1_knee.*t2.*t9.*(5.0./1.6e+1);
t38 = dq2_knee.*t3.*t10.*(5.0./1.6e+1);
t39 = t15.*1.839375e+1;
t40 = t16.*1.839375e+1;
t29 = t17.*(5.0./8.0);
t30 = t18.*(5.0./8.0);
t31 = t19.*(5.0./8.0);
t32 = t20.*(5.0./8.0);
t41 = -t37;
t42 = -t38;
t43 = -t39;
t44 = -t40;
t45 = t19.*(9.81e+2./1.6e+2);
t46 = t20.*(9.81e+2./1.6e+2);
t33 = dq1_knee.*t29;
t34 = dq2_knee.*t30;
t35 = dq1_knee.*t31;
t36 = dq2_knee.*t32;
t47 = -t45;
t48 = -t46;
t49 = t23+t29;
t50 = t24+t30;
t51 = t27+t31;
t52 = t28+t32;
t53 = dq1_hip.*t49;
t54 = dq2_hip.*t50;
t55 = dq1_hip.*t51;
t56 = dq2_hip.*t52;
H = [-dq1_hip.*(t35+t55+dq_torso.*t51)-dq2_hip.*(t36+t56+dq_torso.*t52)-dq_torso.*(t35+t36+t55+t56+dq_torso.*(t4.*(5.0./2.0)+t51+t52))-dq1_knee.*t9.*t19.*(5.0./8.0)-dq2_knee.*t10.*t20.*(5.0./8.0);-dq_torso.*(t33+t34+t53+t54+dq_torso.*(t49+t50+cos(q_torso).*(5.0./2.0)))-dq1_hip.*(t33+t53+dq_torso.*t49)-dq2_hip.*(t34+t54+dq_torso.*t50)-dq1_knee.*t9.*t17.*(5.0./8.0)-dq2_knee.*t10.*t18.*(5.0./8.0)+3.4335e+2;t4.*(-9.81e+2./4.0e+1)+t25+t26+t41+t42+t43+t44+t47+t48-dq_torso.*(dq1_knee.*t2.*(5.0./1.6e+1)+dq2_knee.*t3.*(5.0./1.6e+1));t25+t41+t43+t47-dq1_knee.*dq_torso.*t2.*(5.0./1.6e+1);t47+dq1_hip.*t2.*t5.*(5.0./1.6e+1)+dq_torso.*t2.*t5.*(5.0./1.6e+1);t26+t42+t44+t48-dq2_knee.*dq_torso.*t3.*(5.0./1.6e+1);t48+dq2_hip.*t3.*t6.*(5.0./1.6e+1)+dq_torso.*t3.*t6.*(5.0./1.6e+1)];