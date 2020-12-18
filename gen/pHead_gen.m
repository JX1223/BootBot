function pHead = pHead_gen(in1)
%PHEAD_GEN
%    PHEAD = PHEAD_GEN(in1)
%    Outputs position of head

q_torso = in1(3,:);
x = in1(1,:);
y = in1(2,:);
pHead = [x+0.5.*sin(q_torso);y+0.5.*cos(q_torso)];