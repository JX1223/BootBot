function pSw = pSw_gen(in1)
%PSW_GEN
%    PSW = PSW_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    26-Nov-2020 23:22:10

q2_hip = in1(6,:);
q2_knee = in1(7,:);
q_torso = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = pi./2.0;
t3 = -t2;
t4 = q2_hip+q_torso+t3;
t5 = q2_knee+t4;
pSw = [x+cos(t4)./2.0+cos(t5)./2.0;y-sin(t4)./2.0-sin(t5)./2.0];