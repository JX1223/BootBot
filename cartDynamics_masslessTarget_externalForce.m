function ds = cartDynamics_masslessTarget_externalForce(t,state,Fext)
global mcart

m = mcart;%10;

x = state(1);
dx = state(2);

d2x = Fext(1)/m;
ds = [dx;d2x];

end



