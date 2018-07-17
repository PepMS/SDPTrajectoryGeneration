function tasksFilled = taskFillMatrices(tasksTable, robot, qC, qC_d, ...
    rPos, rPos_d, rPos_dd, rOri, rOri_d, rOri_dd, ...
    jointsLLim, jointsULim, obstacle)
    
    tasksFilled = [];
    for i = 1:length(tasksTable)
        task = tasksTable(i);
        if strcmp(task.name, 'Joints_Normalization')
            task.A = 0.1*eye(robot.n);
            task.b = zeros(robot.n, 1);
        elseif strcmp(task.name, 'Position_Tracking')
            J = robot.jacob0(qC);
            J(3:6, :) = []; % Only jacobian of the position
            J_d = robot.jacob_dot(qC, qC_d);
            J_d(3:6, :) = [];
            
            Tpos = robot.fkine(qC); % Transf. mat. of the end-eff. pose
            pos = Tpos.t(1:2);
            % pos = Tpos(1:2, 4);
            pos_d = J*qC_d;
            G11 = [6; 11]; G21 = [9; 30.25];
            %G11 = [1; 10]; G21 = [1; 10];
            
            task.A = J;
            task.b = -J_d + rPos_dd + G11.*(rPos_d - pos_d) + G21.*(rPos - pos);
        elseif strcmp(task.name, 'Orientation_Tracking')
            qSum = sum(qC);
            J = -sin(qSum)*ones(1, robot.n);
            
            qSum_d = sum(qC_d); 
            J_d = -cos(qSum)*qSum_d*ones(1, robot.n);
            
            G11 = 5; G21 = 5;
            ori = cos(qSum);
            ori_d = J*qC_d;
            
            task.A = J;
            task.b = -J_d*qC_d + rOri_dd + G11*(rOri_d - ori_d) + G21*(rOri - ori);
        elseif strcmp(task.name, 'Joint_Limits')
            d2r = pi/180;
            jointLLim = jointsLLim * d2r;
            jointULim = jointsULim * d2r;
            
            l1 = 2.5; l2 = 2.5;
            
            b1 = -l1*l2*(qC - jointULim) - (l1 + l2)*qC_d;
            b2 = -l1*l2*(-qC + jointLLim) + (l1 + l2)*qC_d;
            task.A = [eye(robot.n); -eye(robot.n)];
            task.b = [b1; b2];
        elseif strcmp(task.name, 'Obstacle_Avoidance')
            
            J = robot.jacob0(qC);
            J(3:6, :) = []; % Only jacobian of the position
            J(:, task.avoidJoint + 1:end) = []; 
            J_der = jacobianDerviative(robot, qC, task.avoidJoint);
                 
            [T, Tpos] = robot.fkine(qC); % Transf. matrices for all joints
            % Matlab 2013
            posT = Tpos(task.avoidJoint); % Tansf. for a specific joint
            pos = posT.t(1:2);
            % Matlab 2017
%             posT = Tpos(:, :, task.avoidJoint); % Tansf. for a specific joint
%             pos = posT(1:2, 4);
            
            err = (pos - obstacle.center);
            tol = 0.1;
            
            fErr = -err'*err + (obstacle.radius + tol)^2;
            derTerm = [];
            for ii = 1:task.avoidJoint 
                derTerm = [derTerm; err'*J_der(:, :, ii)];
            end
            
            fErr_dj = -2*err'*J;
            fErr_djdj = -2*(J'*J + derTerm);
            
            l1 = 10; l2 = 10;
            task.A = [fErr_dj, zeros(length(fErr_dj(:,1)),robot.n - task.avoidJoint)];
            task.b = -l1*l2*fErr - (qC_d(1:task.avoidJoint )'*fErr_djdj + (l1 + l2)*fErr_dj)*qC_d(1:task.avoidJoint );
        elseif strcmp(task.name, 'Manipulability')
            d2r = pi/180;
            J = robot.jacob0(qC);
            J(3:6, :) = []; % Only jacobian of the position
            
            fMan = 1/sqrt(det(J*J'));
            fMan_dj = manipulabilityJacobian(robot, qC);
            fMan_djdj = manipulabilityHessian(robot, qC, fMan_dj);     
            
            l1 = 2.5; l2 = 0.25;                    
            task.A = fMan_dj;
            task.b = -l1*l2*fMan - (qC_d'*fMan_djdj + (l1 + l2)*fMan_dj)*qC_d;
        end
        
        tasksFilled = [tasksFilled, task];
    end
end