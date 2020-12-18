function ds = combinedDynamics(t,s,C)
target_type = 'b';
% global mcart
global kickStart kickEnd pos_est vel_est

% Compute ball estimates


% disp('Combined')
% State of robot
s_robot = s(1:14);


% State of target
s_target = s(15:end);



% collisionDetection = detectCollision(s_robot,s_target,'p');
% isCollided = collisionDetection(1);
% pCol = collisionDetection(2:3);
switch target_type
    %% Cart
    case 'c'
        % disp('u')
        u = controller(t,s_robot,C);
        % disp('Fext')
        Fext = cartDynamics_masslessTarget_forceCalculator(t,s,u);
        if Fext(1) ~=0
            kickStart = 1;
            kickEnd = 0;
        end
        if Fext(1) == 0 && kickStart ==1
            kickEnd = 1;
        end

        Jext = JSw_gen(s_robot);
        pSw = pSw_gen(s_robot);
        % if t > .15
        %     ds_robot = five_link_dynamics_2(t,s_robot);
        % else
        %     ds_robot = five_link_dynamics_external_input(t,s_robot,-Fext,Jext);
        % end
        if kickStart ~=1 || kickEnd ==1
        %     disp("Five Link 2")
            ds_robot = five_link_dynamics_2(t,s_robot);
        else
            ds_robot = five_link_dynamics_external_input(t,s_robot,-Fext,Jext);
        end
        % disp("Target")
        ds_target = cartDynamics_masslessTarget_externalForce(t,s_target,Fext);
        ds = [ds_robot;ds_target];
        % disp("ode")

%% Ball   
    case 'b'
        %u = controller(t,s_robot);
        [Jext, Fext] = ball_calculator(t,s);
        
        if Fext(1) ~=0
            kickStart = 1;
            kickEnd = 0;
        end
        if Fext(1) == 0 && kickStart ==1
            kickEnd = 1;
        end

        %pSw = pSw_gen(s_robot);
        % if t > .15
        %     ds_robot = five_link_dynamics_2(t,s_robot);
        % else
        %     ds_robot = five_link_dynamics_external_input(t,s_robot,-Fext,Jext);
        % end
        if kickStart ~=1 || kickEnd ==1
            ds_robot = five_link_dynamics_2(t,s_robot,C);
        else
            ds_robot = five_link_dynamics_external_input(t,s_robot,-Fext,Jext,C);
        end
        ds_target = ballDynamics_externalForce(t,s_target,Fext);
        if (s_target(1) > 5.2 || s_target(2) > 2.2)
            ds_target = zeros(6,1);
        end
        ds = [ds_robot;ds_target];

    end
end