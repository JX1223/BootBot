function pFoot2 = pFoot2_gen(in1)
%PFOOT2_GEN
%    PFOOT2 = PFOOT2_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.

%    21-Nov-2020 23:39:50


q2_hip = in1(6,:);
q2_knee = in1(7,:);
q_torso = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = pi./2.0;
t3 = -t2;
t4 = q2_hip+q_torso+t3;
t5 = q2_knee+t4;
pFoot2 = [x+cos(t4)./2.0+cos(t5)./2.0;y-sin(t4)./2.0-sin(t5)./2.0];
