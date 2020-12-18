% Animate the Five-Link Walker
% Modified from animateThreeLink (ME193b)
% Jason Xu

function F = animateFiveLinkInterp_withBall(tData, sData, fps, C)
    % Inputs: tData, qData: arrays from ODE. fps: frames per second
    % Ouputs: F: array of frames of animation, can be used with movie()
    % animateFiveLink takes in states of the 5-link robot and interpolates
    % based on a desired fps. Plays movie once.
    % sData should be the combined state vector of the 5-link and ball
    addpath('./gen');
    dt = 1 / fps; % Calculate even time interval based on FPS
    t = tData(1):dt:tData(end); % Generate even time intervals

    % Extract 5-link and ball states from combined state vector
    s_5link = interp1(tData, sData(:,1:14), t); % Generate interpolated states, 5-link state is from 1 to 14 
    s_ball = interp1(tData, sData(:,15:20), t); % Generate interpolated states, ball state is from 15 to 20 
    % Create initial figure
    f = figure(1000);
    clf;
    line([-1, 20],[0;0],'Color', 'k', 'LineWidth', 2)
    axis equal;
    axis([-1 5 -1 2]); 
    grid on ;
    
    % Get 5-link points
    x = s_5link(:,1)';
    y = s_5link(:,2)';
    pHead = pHead_gen(s_5link');
    pKnee1 = pKnee1_gen(s_5link');
    pKnee2 = pKnee2_gen(s_5link');
    pFoot1 = pFoot1_gen(s_5link');
    pFoot2 = pFoot2_gen(s_5link');
    
    % Get ball points
    x_ball = s_ball(:,1)';
    y_ball = s_ball(:,2)';
    dx_ball = s_ball(:,4)';
    dy_ball = s_ball(:,5)';
   
    % Initialize links
    % l1 is torso
    l1 = line([x(1); pHead(1,1)], [y(1), pHead(2,1)], 'Color', 'k', 'LineWidth', 2);
    % l2 is leg 1 femur
    l2 = line([x(1); pKnee1(1,1)], [y(1); pKnee1(2,1)], 'Color', 'r', 'LineWidth', 2);
    % l3 is leg 1 tibia
    l3 = line([pKnee1(1,1), pFoot1(1,1)], [pKnee1(2,1), pFoot1(2,1)], 'Color', 'r', 'LineWidth', 2);
    % l4 is leg 2 femur
    l4 = line([x(1); pKnee2(1,1)], [y(1); pKnee2(2,1)], 'Color', 'b', 'LineWidth', 2);
    % l5 is leg 2 tibia
    l5 = line([pKnee2(1,1), pFoot2(1,1)], [pKnee2(2,1), pFoot2(2,1)], 'Color', 'b', 'LineWidth', 2);
    
    hold on;
    
    % Initialize LIDAR visualization
    angles = linspace(-C.lidar_FOV/2, C.lidar_FOV/2, 100)';
    q_torso = s_5link(1,3);
    angles = angles - q_torso;
    X = [pHead(1); C.lidar_range*cos(angles) + pHead(1)];
    Y = [pHead(2); C.lidar_range*sin(angles) + pHead(2)];
    fan = fill(X, Y, 'red', 'FaceAlpha', 0.1, 'EdgeAlpha', 0);
    
    % Initialize joint marks
    % Mark hip
    p1 = plot(x(1), y(1), 'ko', 'MarkerSize',7,'MarkerEdgeColor','k','MarkerFaceColor','g');
    % Mark knee joints
    p2 = plot(pKnee1(1,1), pKnee1(2,1), 'bo', 'MarkerSize',7,'MarkerEdgeColor','k','MarkerFaceColor','g');
    p3 = plot(pKnee2(1,1), pKnee2(2,1), 'ro', 'MarkerSize',7,'MarkerEdgeColor','k','MarkerFaceColor','g');
    % Mark feet
    p4 = plot(pFoot1(1,1), pFoot1(2,1), 'bo', 'MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','g');
    p5 = plot(pFoot2(1,1), pFoot2(2,1), 'ro', 'MarkerSize',7,'MarkerEdgeColor','b','MarkerFaceColor','g');
    % Mark head
    p6 = plot(pHead(1,1), pHead(2,1), 'ko', 'MarkerSize',7,'MarkerEdgeColor','k','MarkerFaceColor','g');
    
    % Intialize ball
    ball = rectangle('Position',[x_ball(1)-C.ball_radius, y_ball(1)-C.ball_radius, 2*C.ball_radius, 2*C.ball_radius], 'Curvature', [1 1], 'LineWidth', 1);
    ball_vel = quiver(x_ball(1), y_ball(1), dx_ball(1), dy_ball(1));
    
    % Update graphics at each frame
    F(length(t)) = struct('cdata',[],'colormap',[]);
    for i =1:length(t) 
        % Update 5-link
        set(l1, 'XData', [x(i); pHead(1,i)], 'YData', [y(i), pHead(2,i)]);
        set(l2, 'XData', [x(i); pKnee1(1,i)], 'YData', [y(i); pKnee1(2,i)]);
        set(l3, 'XData', [pKnee1(1,i), pFoot1(1,i)], 'YData', [pKnee1(2,i), pFoot1(2,i)]);
        set(l4, 'XData', [x(i); pKnee2(1,i)], 'YData', [y(i); pKnee2(2,i)]);
        set(l5, 'XData', [pKnee2(1,i), pFoot2(1,i)], 'YData', [pKnee2(2,i), pFoot2(2,i)]);
        
        set(p1, 'XData', x(i), 'YData', y(i));
        set(p2, 'XData', pKnee1(1,i), 'YData', pKnee1(2,i));
        set(p3, 'XData', pKnee2(1,i), 'YData', pKnee2(2,i));
        set(p4, 'XData', pFoot1(1,i), 'YData', pFoot1(2,i));
        set(p5, 'XData', pFoot2(1,i), 'YData', pFoot2(2,i));
        set(p6, 'XData', pHead(1,i), 'YData', pHead(2,i));
        
        % Update LIDAR visual
        angles = linspace(-C.lidar_FOV/2, C.lidar_FOV/2, 100)';
        q_torso = s_5link(i,3);
        angles = angles - q_torso;
        X = [pHead(1,i); C.lidar_range*cos(angles) + pHead(1,i)];
        Y = [pHead(2,i); C.lidar_range*sin(angles) + pHead(2,i)];
        set(fan, 'Vertices', [X, Y]);
        
        % Update ball visuals
        set(ball, 'Position', [x_ball(i) - C.ball_radius, y_ball(i) - C.ball_radius, 2*C.ball_radius, 2*C.ball_radius]);
        set(ball_vel, 'XData', x_ball(i), 'YData', y_ball(i), 'UData', dx_ball(i), 'VData', dy_ball(i));
        
        % Update figure, store in frame
        drawnow update;
        F(i) = getframe(f);
    end
    hold off;
    %movie(f, F, 1, fps);
end