function [qT, qT_d, qT_dd] = solveSDPSProblem(robot, tasksTable, q0, q0_d, ...
            rPos, rPos_d, rPos_dd, rOri, rOri_d, rOri_dd, ...
            jointsLBound, jointsUBound, obstacle, simLength, dt)
    qT = [];
    qT_d = [];
    qT_dd = [];
    
    qC = q0;
    qC_d = q0_d;
    % i = 1:simLength
    for i = 1:simLength
        tic
        % Store current position for joints
        qT_d =  [qT_d, qC_d];
        qT =  [qT, qC];
        
        % ------Solving Hierarchy problem
        
        maxPriority = taskGetMaxPriority(tasksTable);
        currPriority = tasksTable(1).priority;
        tasksTable = taskFillMatrices(tasksTable, robot, qC, qC_d, ...
            rPos(:, i), rPos_d(:, i), rPos_dd(:, i), ...
            rOri(i), rOri_d(i), rOri_dd(i),...
            jointsLBound, jointsUBound, obstacle);
        
        A_c = []; % Matrix for equality constraints
        b_c = []; % Vector for equality constraints
        C_c = []; % Matrix for inequality constraints
        d_c = []; % Vecotr for inequality constraints
        
                
        % Demonstration for 
        task = tasksTable(1); J1 = task.A; b1 = task.b; % Position
        task = tasksTable(2); J2 = task.A; b2 = task.b; % Orientation       

         % < LMI >
         H1 = J1'*J1 + eye(4);
         g1 = -2*(J1'*b1);
         bt = -[0 0 0 0 1]';
         % F1 = [gamma-g2'*x, x'*H2'; H2*x, eye(4)];
         F0 = blkdiag(0, eye(2));
         F1 = [-g1(1), J1(:, 1)'; J1(:, 1), zeros(2)];
         F2 = [-g1(2), J1(:, 2)'; J1(:, 2), zeros(2)];
         F3 = [-g1(3), J1(:, 3)'; J1(:, 3), zeros(2)];
         F4 = [-g1(4), J1(:, 4)'; J1(:, 4), zeros(2)];
         F5 = blkdiag(1, zeros(2));
         
         ct = vec(F0);
         At = [];
         At(:, 1) = -vec(F1);
         At(:, 2) = -vec(F2);
         At(:, 3) = -vec(F3);
         At(:, 4) = -vec(F4);
         At(:, 5) = -vec(F5);
         K = [];
         K.s = size(F0, 1);
         pars.fid = 0;
         [x,y,info] = sedumi(At,bt,ct,K, pars);
         qC_dd_1 = y(1:4);
         
         if (i==287)
             aaaa = 4;
         end
         
         H2 = J2'*J2 + eye(4);
         g2 = -2*(J2'*b2);
         bt = -[0 0 0 0 1]';
         % F1 = [gamma-g2'*x, x'*H2'; H2*x, eye(4)];
         F0 = blkdiag(0, eye(1));
         F1 = [-g2(1), J2(:, 1)'; J2(:, 1), zeros(1)];
         F2 = [-g2(2), J2(:, 2)'; J2(:, 2), zeros(1)];
         F3 = [-g2(3), J2(:, 3)'; J2(:, 3), zeros(1)];
         F4 = [-g2(4), J2(:, 4)'; J2(:, 4), zeros(1)];
         F5 = blkdiag(1, zeros(1));
         
         ctt = [-[J1*qC_dd_1; -J1*qC_dd_1];vec(F0)];
         At = [];
         At(:, 1) = -vec(F1);
         At(:, 2) = -vec(F2);
         At(:, 3) = -vec(F3);
         At(:, 4) = -vec(F4);
         At(:, 5) = -vec(F5);
         J1_aux = [J1, [0;0]];
         A = [J1_aux; -J1_aux];
         Att = [-A; At];
         K = [];
         K.l = size(A,1);
         K.s = size(F0, 1);
         pars.fid = 0;
         [x,y,info] = sedumi(Att,bt,ctt,K, pars);
         
         qC_dd = y(1:4);
         % </ LMI >
         
         % /------Solving Hierarchy problem
        
        % Computing Euler integration
        qC_d_aux = qC_d + qC_dd*dt;
        qC_aux = qC + qC_d*dt + 1/2*qC_dd*dt^2;

        qC_d = qC_d_aux;
        qC = qC_aux;

        % Store current joint acceleration
        qT_dd =  [qT_dd, qC_dd];
        tSolving = toc;
        
        if mod(i, 10) == 0
            display(strcat('Iteration number: ', num2str(i), '; Time: ', num2str(tSolving)));
        end
    end   
end
