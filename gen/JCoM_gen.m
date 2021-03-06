function JCoM = JCoM_gen(in1)
%JCOM_GEN
%    JCOM = JCOM_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    07-Dec-2020 20:34:49

q1_hip = in1(4,:);
q2_hip = in1(6,:);
q1_knee = in1(5,:);
q2_knee = in1(7,:);
q_torso = in1(3,:);
t2 = pi./2.0;
t3 = -t2;
t4 = q1_hip+q_torso+t3;
t5 = q2_hip+q_torso+t3;
t6 = cos(t4);
t7 = cos(t5);
t8 = q1_knee+t4;
t9 = q2_knee+t5;
t10 = sin(t4);
t11 = sin(t5);
t12 = cos(t8);
t13 = cos(t9);
t14 = sin(t8);
t15 = sin(t9);
t16 = t6.*(3.0./1.4e+2);
t17 = t7.*(3.0./1.4e+2);
t18 = t10.*(3.0./1.4e+2);
t19 = t11.*(3.0./1.4e+2);
t20 = t12./1.4e+2;
t21 = t13./1.4e+2;
t22 = t14./1.4e+2;
t23 = t15./1.4e+2;
t24 = -t16;
t25 = -t17;
t26 = -t18;
t27 = -t19;
t28 = -t20;
t29 = -t21;
t30 = -t22;
t31 = -t23;
JCoM = reshape([4.0./7.0,0.0,0.0,4.0./7.0,t26+t27+t30+t31+cos(q_torso)./1.4e+2,t24+t25+t28+t29-sin(q_torso)./1.4e+2,t26+t30,t24+t28,t30,t28,t27+t31,t25+t29,t31,t29],[2,7]);
