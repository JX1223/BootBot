function dCoM = robot_dCoM_gen(in1)
%ROBOT_DCOM_GEN
%    DCOM = ROBOT_DCOM_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    07-Dec-2020 20:34:50

dq1_hip = in1(11,:);
dq2_hip = in1(13,:);
dq1_knee = in1(12,:);
dq2_knee = in1(14,:);
dq_torso = in1(10,:);
dx = in1(8,:);
dy = in1(9,:);
q1_hip = in1(4,:);
q2_hip = in1(6,:);
q1_knee = in1(5,:);
q2_knee = in1(7,:);
q_torso = in1(3,:);
t2 = q1_hip+q_torso;
t3 = q2_hip+q_torso;
t4 = cos(t2);
t5 = cos(t3);
t6 = q1_knee+t2;
t7 = q2_knee+t3;
t8 = sin(t2);
t9 = sin(t3);
t10 = cos(t6);
t11 = cos(t7);
t12 = sin(t6);
t13 = sin(t7);
dCoM = [dx+dq1_hip.*t4.*(3.0./5.6e+1)+dq2_hip.*t5.*(3.0./5.6e+1)+(dq1_hip.*t10)./5.6e+1+(dq2_hip.*t11)./5.6e+1+(dq1_knee.*t10)./5.6e+1+(dq2_knee.*t11)./5.6e+1+dq_torso.*t4.*(3.0./5.6e+1)+dq_torso.*t5.*(3.0./5.6e+1)+(dq_torso.*t10)./5.6e+1+(dq_torso.*t11)./5.6e+1+(dq_torso.*cos(q_torso))./1.4e+1;dy-dq1_hip.*t8.*(3.0./5.6e+1)-dq2_hip.*t9.*(3.0./5.6e+1)-(dq1_hip.*t12)./5.6e+1-(dq2_hip.*t13)./5.6e+1-(dq1_knee.*t12)./5.6e+1-(dq2_knee.*t13)./5.6e+1-dq_torso.*t8.*(3.0./5.6e+1)-dq_torso.*t9.*(3.0./5.6e+1)-(dq_torso.*t12)./5.6e+1-(dq_torso.*t13)./5.6e+1-(dq_torso.*sin(q_torso))./1.4e+1];