function [qT, qT_d, qT_dd] = solveQPProblem(robot, tasksTable, q0, q0_d, ...
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
        
        while currPriority <= maxPriority
            
            tasksPriority = taskGetSamePriority(currPriority, tasksTable);
            ineqDim = taskGetInequalityDimension(tasksPriority);
            eqDim = taskGetEqualityDimension(tasksPriority);
            
            H = 0.0*eye(robot.n); % Hessian matrix of the LSP (with a reg. Factor)
            % H = [];
            g = zeros(robot.n,1); % Hessian matrix of the LSP
            A = []; % A matrices of the Ob. Fun.
            b = []; % b vectors of the Ob. Fun.
            C = [];
            d = [];
            
            for ii = 1:length(tasksPriority)
                task = tasksPriority(ii);
                if task.equality == 1
                    A = [A; task.A];
                    b = [b; task.b];
                    H = H + (task.A'*task.A);
                    g = g + (-2*task.A'*task.b);
                elseif task.equality == 0
                    C = [C; task.A];
                    d = [d; task.b];
                end                
            end
            
             % Adding the slack varaibles to the OF
            H = blkdiag(H, eye(ineqDim));
            g = [g; zeros(ineqDim,1)];
                                    
            % inequality of the current priority
            C_p = [C, -eye(ineqDim)]; % Adding the slack variables to the inequalities
            d_p_ub = d;
            % Inequalities from the past priorities
            C_p = [C_p; C_c];
            d_p_ub = [d_p_ub; d_c];
            % Equalities from past priorities
            C_p = [C_p; A_c];
            d_p_lb = -1000*ones(length(d_p_ub), 1);
            d_p_ub = [d_p_ub; b_c];
            d_p_lb = [d_p_lb; b_c];
                        
            % Solve the problem with qPOASES
            options = qpOASES_options( 'enableRegularisation', 1, 'epsRegularisation', 0.01);
            [xOpt, fval, exitf, iter, lambda, auxOutput] = qpOASES(H, g, C_p, [], [], d_p_lb, d_p_ub, options);
            %[xOpt, fval, exitf, iter, lambda, auxOutput] = qpOASES(H, g, C_p, [], [], d_p_lb, d_p_ub);
            
            qC_dd = xOpt(1:robot.n);
            w_opt = [];
            if length(xOpt) > robot.n
                w_opt = xOpt(robot.n + 1:end);
            end
            
            % Refreshing the constraints
            if ~ isempty(A)
                A_c = [A_c; A];
                b_c = [b_c; A*qC_dd];
            end
            
            if ~ isempty(C)
                for ii = 1:length(C(:, 1))
                    if C(ii, :)*qC_dd <= d(ii, :)
                        C_c = [C_c; C(ii, :)];
                        d_c = [d_c; d(ii)];
                    else
                        A_c = [A_c; C(ii, :)];
                        b_c = [b_c; C(ii, :)*qC_dd];
                    end
                end            
            end
            currPriority = currPriority + 1;
        end
        
        % Demonstration for 
%         task = tasksTable(1); J1 = task.A; b1 = task.b;
%         task = tasksTable(2); J2= task.A; b2 = task.b;
        
        % < Mixed systems >
%         H = J2'*J2 + eye(4);
%         g = -2*J2'*b2;
%         
%         q1 = pinv(J1)*b1;
%         C_p = J1;
%         d_p_lb = J1*q1;
%         d_p_ub = d_p_lb;   
%         
%          [xOpt, fval, exitf, iter, lambda, auxOutput] = qpOASES(H, g, C_p, [], [], d_p_lb, d_p_ub);
%          qC_dd = xOpt;
         
         % </ Mixed Systems >
         
         % < Analytical >
%          qC_dd = pinv(J1)*b1 + pinv(J2*(eye(4) - pinv(J1)*J1))*(b2 - J2*pinv(J1)*b1);
         % </ Analytical >
         
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
