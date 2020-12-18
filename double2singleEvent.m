function [value, isterminal, direction] = double2singleEvent(t,x)
%     u = controller(t,x);
%     global Fext
%     Jext = JpComTorso_gen(x);
%     JFext = Jext'*Fext;
%     FSt_ = FSt2_gen(x,u,JFext);
    footCheck = pSt2_gen(x);
%     value = [FSt_(2); FSt_(4)];
%     isterminal = [1; 1];
%     direction = [-1; -1];
    tolerance = 0.01;
    value = footCheck(4)-tolerance;
    isterminal = 1;
    direction = 1;
end

