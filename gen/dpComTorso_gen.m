function dpComTorso = dpComTorso_gen(in1)
%DPCOMTORSO_GEN
%    DPCOMTORSO = DPCOMTORSO_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    23-Nov-2020 19:42:38

dq_torso = in1(10,:);
dx = in1(8,:);
dy = in1(9,:);
q_torso = in1(3,:);
dpComTorso = [dx+(dq_torso.*cos(q_torso))./4.0;dy-(dq_torso.*sin(q_torso))./4.0];