function manDer = manipulabilityJacobian(robot, qC)
    manDer = [];
    step = 0.05;
    J = robot.jacob0(qC);
    J(3:6, :) = [];
    man = 1/sqrt(det(J*J'));
    for ii = 1:robot.n
        qC_step = qC;
        qC_step(ii) = qC_step(ii) + step;
        J_step = robot.jacob0(qC_step);
        J_step(3:6, :) = [];
        man_step = 1/sqrt(det(J_step*J_step'));
        manDer = [manDer, (man_step - man)/step]; 
    end
end