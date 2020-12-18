function relabel_q = relabel_q_gen(in1)
%RELABEL_Q_GEN
%    RELABEL_Q = RELABEL_Q_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    21-Nov-2020 23:52:31

q1_hip = in1(4,:);
q2_hip = in1(6,:);
q1_knee = in1(5,:);
q2_knee = in1(7,:);
q_torso = in1(3,:);
x = in1(1,:);
y = in1(2,:);
relabel_q = [x;y;q_torso;q2_hip;q2_knee;q1_hip;q1_knee];