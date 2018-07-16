function taskPlot(tasksTable, robot, qT, refPos, refOri, ...
    lowBound, uppBound, obstacle, avoidanceJoints, colors)
    for i = 1:length(tasksTable)
        task = tasksTable(i);
        if strcmp(task.name, 'Position_Tracking') && task.plot
            taskPlotPosTracking(robot, qT, refPos, obstacle, avoidanceJoints, colors);
        elseif strcmp(task.name, 'Orientation_Tracking') && task.plot
            taskPlotOriTracking(qT, refOri, colors);
        elseif strcmp(task.name, 'Joint_Limits') && task.plot
            taskPlotJointLimits(robot, qT, lowBound, uppBound, colors);
        end
    end
end