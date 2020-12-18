function CoM = robot_CoM_gen_2(in1)
%ROBOT_COM_GEN_2
%    COM = ROBOT_COM_GEN_2(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    07-Dec-2020 20:34:48

q1_hip = in1(4,:);
q2_hip = in1(6,:);
q1_knee = in1(5,:);
q2_knee = in1(7,:);
q_torso = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = pi./2.0;
t3 = -t2;
t4 = q1_hip+q_torso+t3;
t5 = q2_hip+q_torso+t3;
t6 = q1_knee+t4;
t7 = q2_knee+t5;
CoM = [x.*(4.0./7.0)+cos(t4).*(3.0./1.4e+2)+cos(t5).*(3.0./1.4e+2)+cos(t6)./1.4e+2+cos(t7)./1.4e+2+sin(q_torso)./1.4e+2;y.*(4.0./7.0)+cos(q_torso)./1.4e+2-sin(t4).*(3.0./1.4e+2)-sin(t5).*(3.0./1.4e+2)-sin(t6)./1.4e+2-sin(t7)./1.4e+2];