function manHess = manipulabilityHessian(robot, qC, manJac)
    manHess = [];
    step = 0.05;
    for ii = 1:robot.n
        qC_step = qC;
        qC_step(ii) = qC_step(ii) + step;
        manJac_step = manipulabilityJacobian(robot, qC_step);
        manHess = [manHess, (manJac_step' - manJac')/step]; 
    end
end