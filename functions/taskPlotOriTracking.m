function taskPlotOriTracking(qT, refOri, colors)
    figOriTracking = figure;
    
    r2d = 180/pi;
    
    plot(r2d*(sum(qT))         , 'linewidth', 1.5, 'Color', colors{1}, 'DisplayName', 'End-Effector');
    hold on;
    plot(r2d*acos(refOri(:, 1)), 'LineStyle', ':', 'linewidth', 1.5, 'Color', colors{2}, 'DisplayName', 'Reference');
    title('Orientation tracking'); legend;
    xlabel('samples'); ylabel('Orientation [deg]');
    ylim([0 95])
end