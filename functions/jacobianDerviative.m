function J_der = jacobianDerviative(robot, qC, avoidJoint)
    
    step = 0.05;
    J_der = [];
    J = robot.jacob0(qC);
    J(3:6, :) = [];
    J(:, avoidJoint + 1:end) = [];
        
    for i = 1:avoidJoint
        qC_step = qC;
        qC_step(i) = qC_step(i) + step;
        J_step = robot.jacob0(qC_step);
        J_step(3:6, :) = [];
        J_step(:, avoidJoint + 1:end) = [];
        J_der(:, :, i) = (J - J_step) / -step;
    end

end