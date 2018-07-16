function scenarioPlot(robot, ref, obs)
    
    qZ = zeros(robot.n, 1);
    robot.plot(qZ');
    
    if ~isempty(ref)              
        hold on
        plot3(ref(:, 1),ref(:, 2),zeros(length(ref),1),'g.', 'linewidth', 3)
    end
    
    if ~isempty(obs)
        hold on
        [X, Y, Z] = cylinder(obs.radius);
        X = X + obs.center(1);
        Y = Y + obs.center(2);
        surf(X, Y, Z, 'FaceAlpha', 0.5);
    end
    
end
