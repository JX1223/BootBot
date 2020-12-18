function dq = dq_gen(in1)
%DQ_GEN
%    DQ = DQ_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    21-Nov-2020 23:20:57

dq1_hip = in1(11,:);
dq2_hip = in1(13,:);
dq1_knee = in1(12,:);
dq2_knee = in1(14,:);
dq_torso = in1(10,:);
dx = in1(8,:);
dy = in1(9,:);
dq = [dx;dy;dq_torso;dq1_hip;dq1_knee;dq2_hip;dq2_knee];
