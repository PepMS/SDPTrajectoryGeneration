function taskPlot(tasksTable, solTime, plotTime, robot, qT, refPos, refOri, ...
                  lowBound, uppBound, obstacle, avoidanceJoints, colors)
    
    for i = 1:length(tasksTable)
        task = tasksTable(i);
        if strcmp(task.name, 'Position_Tracking') && task.plot
            taskPlotPosTracking(robot, qT, refPos, obstacle, avoidanceJoints, colors);
        elseif strcmp(task.name, 'Orientation_Tracking') && task.plot
            taskPlotOriTracking(qT, refOri, colors);
        elseif strcmp(task.name, 'Joint_Limits') && task.plot
            taskPlotJointLimits(robot, qT, lowBound, uppBound, colors);
        elseif strcmp(task.name, 'Obstacle_Avoidance') && task.plot
            taskPlotObsAvoidance(robot, qT, refPos, obstacle, avoidanceJoints, colors);
        end
    end
    
    if plotTime
        figure;
        plot(solTime, 'LineWidth', 1.5, 'Color', colors{1}, 'DisplayName', 'It. Time');
        grid on
        xlabel('iteration'); ylabel('Solving time [s]');
        title('Solving time for each iteration');
        xlim([0, length(solTime)]);
        ylim([0, max(solTime)+0.05]);
        hold on
        plot(mean(solTime)*ones(1, length(solTime)),'LineStyle',':','LineWidth', 1.5, 'Color', colors{2}, 'DisplayName', 'Average');
        legend
    end
    
end