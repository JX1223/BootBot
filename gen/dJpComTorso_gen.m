function dJpComTorso = dJpComTorso_gen(in1)
%DJPCOMTORSO_GEN
%    DJPCOMTORSO = DJPCOMTORSO_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    07-Dec-2020 20:06:56

dq_torso = in1(10,:);
q_torso = in1(3,:);
dJpComTorso = reshape([0.0,0.0,0.0,0.0,dq_torso.*sin(q_torso).*(-1.0./4.0),dq_torso.*cos(q_torso).*(-1.0./4.0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[2,7]);
