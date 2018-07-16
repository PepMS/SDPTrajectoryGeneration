function taskPlotJointLimits(robot, qT, lowBound, uppBound, colors)
    figPosTracking = figure;
    r2d = 180/pi;
    for i = 1:robot.n
        subplot(robot.n, 1, i)
        plot(qT(i, :)*r2d, 'Color', colors{1}, 'linewidth', 1.5, 'DisplayName', strcat('Joint ', i)); hold on
        if lowBound(i) ~= -1000
            plot(lowBound(i)*ones(1, length(qT)), 'LineStyle', ':', 'Color', colors{2}, 'linewidth', 1.5, 'DisplayName', 'Lower Bound');
        end
        if uppBound(i) ~= 1000
            plot(uppBound(i)*ones(1, length(qT)), 'LineStyle', ':', 'Color', colors{2}, 'linewidth', 1.5, 'DisplayName', 'Upper Bound');
        end
        xlim([0 length(qT)]);
        legend
        title(strcat('Limits for joint ', int2str(i)))
    end
end