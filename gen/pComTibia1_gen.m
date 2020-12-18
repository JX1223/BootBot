function pComTibia1 = pComTibia1_gen(in1)
%PCOMTIBIA1_GEN
%    PCOMTIBIA1 = PCOMTIBIA1_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    21-Nov-2020 23:39:50

q1_hip = in1(4,:);
q1_knee = in1(5,:);
q_torso = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = pi./2.0;
t3 = -t2;
t4 = q1_hip+q_torso+t3;
t5 = q1_knee+t4;
pComTibia1 = [x+cos(t4)./2.0+cos(t5)./4.0;y-sin(t4)./2.0-sin(t5)./4.0];