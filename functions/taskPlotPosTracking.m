function taskPlotPosTracking(robot, qT, refPos, obstacle, avoidanceJoints, colors)
%% Trajectory XTime plot    
    figPosTracking = figure; 
    
    trajEE = [];
    joints = cell(robot.n);
    for i=1:length(refPos)
        qC = qT(:, i)';
        [T_single, T_fkine] = robot.fkine(qC);
        trajEE = [trajEE, T_fkine(robot.n).t(1:2)];
        for ii = 1:avoidanceJoints-1
            joints{ii} = [joints{ii}, T_fkine(robot.n - ii).t(1:2)];
        end
    end
    
    subplot(2,1,1)
    plot(trajEE(1,:) , 'Color', colors{1},'linewidth', 1.5, 'DisplayName', 'End-Effector'); hold on
    plot(refPos(1, :), 'LineStyle', ':', 'Color', colors{2},'linewidth', 1.5, 'DisplayName', 'Reference');
    title('X axis tracking'); legend
    xlabel('samples'); ylabel('x position [m]'); 
    
    subplot(2,1,2)
    plot(trajEE(2,:) , 'Color', colors{1},'linewidth', 1.5, 'DisplayName', 'End-Effector'); hold on
    plot(refPos(2, :),'LineStyle', ':', 'Color', colors{2},'linewidth', 1.5, 'DisplayName', 'Reference');
    title('Y axis tracking'); legend
    xlabel('samples'); ylabel('y position [m]'); 
    %% Trajectory XY plot
    figTrajectory = figure;

    plot(trajEE(1,:),trajEE(2,:), 'Color', colors{1},'linewidth', 1.5, 'DisplayName', 'End-Effector'); hold on
    for ii = 1:avoidanceJoints-1
        legText = strcat('Joint ', int2str(robot.n - ii));
        plot(joints{ii}(1,:), joints{ii}(2,:), 'Color', colors{3+ii},'linewidth', 1,'DisplayName', legText);
    end
    
    plot(refPos(1, :), refPos(2, :),'g-.','linewidth', 1.5, 'DisplayName', 'Position Reference');
    
    title('Trajectory tracking')
    legend; axis equal
    xlabel('x [m]'); ylabel('y [m]'); 
    if ~ isempty(obstacle)
        r = obstacle.radius;
        center = obstacle.center;
      
        THETA = linspace(0, 2*pi, 1000);
        RHO = ones(1,1000)*r;
        [X,Y] = pol2cart(THETA,RHO);
        X=X+center(1);
        Y=Y+center(2);
        fill(X, Y, 'r','DisplayName', 'Obstacle');
        alpha(0.25);
    end
end