function dJSt = dJSt_gen(in1)
%DJST_GEN
%    DJST = DJST_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    07-Dec-2020 20:06:54

dq1_hip = in1(11,:);
dq1_knee = in1(12,:);
dq_torso = in1(10,:);
q1_hip = in1(4,:);
q1_knee = in1(5,:);
q_torso = in1(3,:);
t2 = q1_hip+q_torso;
t3 = dq1_hip+dq1_knee+dq_torso;
t4 = cos(t2);
t5 = q1_knee+t2;
t6 = sin(t2);
t7 = cos(t5);
t8 = sin(t5);
t9 = (dq1_hip.*t4)./2.0;
t10 = (dq_torso.*t4)./2.0;
t11 = (dq1_hip.*t6)./2.0;
t12 = (dq_torso.*t6)./2.0;
t13 = -t9;
t14 = -t10;
t15 = -t11;
t16 = -t12;
t17 = (dq1_hip.*t7)./2.0;
t18 = (dq1_knee.*t7)./2.0;
t19 = (dq_torso.*t7)./2.0;
t20 = (dq1_hip.*t8)./2.0;
t21 = (dq1_knee.*t8)./2.0;
t22 = (dq_torso.*t8)./2.0;
t23 = -t17;
t24 = -t18;
t25 = -t19;
t26 = -t20;
t27 = -t21;
t28 = -t22;
t29 = t15+t16+t26+t27+t28;
t30 = t13+t14+t23+t24+t25;
dJSt = reshape([0.0,0.0,0.0,0.0,t29,t30,t29,t30,t3.*t8.*(-1.0./2.0),t3.*t7.*(-1.0./2.0),0.0,0.0,0.0,0.0],[2,7]);