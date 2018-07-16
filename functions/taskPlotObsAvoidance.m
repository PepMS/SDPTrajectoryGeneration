function taskPlotObsAvoidance(robot, qT, refPos, obstacle, avoidanceJoints, colors)
    figure;
    
    % Store the value for every joint tip
    trajEE = [];
    joints = cell(robot.n-1);
    for i=1:length(refPos)
        qC = qT(:, i)';
        [T_single, T_fkine] = robot.fkine(qC);
        trajEE = [trajEE, T_fkine(robot.n).t(1:2)];
        for ii = 1:avoidanceJoints-1
            joints{ii} = [joints{ii}, T_fkine(robot.n - ii).t(1:2)];
        end
    end
    
    tol = 0.1;
    err  = cell(1+length(joints), 1);
    err_n = cell(1+length(joints), 1);
    errL = length(err);
    for ii=1:length(err)
        if (ii==1)
            err{errL-ii+1} = trajEE - obstacle.center;
        else
            err{errL-ii+1} = joints{ii-1} - obstacle.center;
        end
        err_n{errL-ii+1} = dot(err{errL-ii+1}, err{errL-ii+1});
        subplot(errL, 1, ii)
        plot(err_n{errL-ii+1}, 'Color', colors{1}, 'LineWidth', 1.5, 'DisplayName', 'Distance')  
        hold on
        plot(ones(length(err_n{errL-ii+1}),1)*(obstacle.radius + tol)^2, 'LineStyle', ':','Color', colors{2}, 'LineWidth', 1.5, 'DisplayName', 'Collision') 
        xlim([0 length(err_n{errL-ii+1})]); ylim([(obstacle.radius + tol)^2 - 0.1, max(err_n{errL-ii+1})+0.1]);
        legend
        title(strcat('Distance from obstacle, Joint ', int2str(errL-ii+1)));
        xlabel('samples'); ylabel('distance [m]');
        grid on
    end
    
end