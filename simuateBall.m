function simulateBall()
    % Define time for simulation
    T_span = [0 10] ;
    
    % Define initial condition for ball
    x0 = [ 5;
           2;
           0;
          -2;
           0;
           0] ;
      
     odeoptions = odeset('Events', @ground_contact) ;
    
    t_vec = [0] ; x_vec = zeros(1,6) ;
    
    % Simulate 5 bounces
    for j = 1:5
        [t x] = ode45(@ballDynamics, T_span, x0, odeoptions) ;
        x_minus = x(end,:)' ;
        x_plus = impact_map(x_minus) ; % Impact dynamics
        x0 = x_plus ;
        T_span = [t(end) 10] ;
        t_vec =  [t_vec(1:end-1) ; t] ;
        x_vec =  [x_vec(1:end-1,:); x] ;
    end    
    
    animateBall(t_vec,x_vec,60);
end

function animateBall(tData,qData,fps)
    dt = 1 / fps; % Calculate even time interval based on FPS
    t = tData(1):dt:tData(end); % Generate even time intervals
    qDataInterp = interp1(tData, qData, t); % Generate interpolated states
    
    addpath('./gen')
    f = figure(1000);
    f.Visible = "off";
    F(length(t)) = struct('cdata',[],'colormap',[]);
    for i =1:length(t) 
        clf ;
        hold on;
        drawBall(qDataInterp(i, :)');
        axis equal;
        line([-1, 20],[0;0],'Color', 'k', 'LineWidth', 2)
        axis([-1 5 -1 2]) ; 
        grid on ;
        drawnow ;
        hold off;
        F(i) = getframe(f);
    end
    f.Visible = "on";
    movie(f, F, 1, fps);
end

function drawBall(q)

   pCoM = ball_CoM_gen(q);
   rad = .3;
        
   viscircles(pCoM',rad, 'LineStyle', ':');
    
end


function x_plus = impact_map(x_minus)
    eta = .8 ;
    
    x_plus = [x_minus(1:4) ;
              -eta*x_minus(5);
              x_minus(6)] ;
end

function [VALUE,ISTERMINAL,DIRECTION] = ground_contact(t, x)

    y = x(2) ;
    rad = .3;
    VALUE = y-rad ; % check for ball position crossing zero
    ISTERMINAL = 1 ; % stop integration when value = 0
    DIRECTION = -1 ; % check for zero crossing from +ve to -ve
end


