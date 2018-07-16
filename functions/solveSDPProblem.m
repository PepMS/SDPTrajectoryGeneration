function [qT, qT_d, qT_dd, tSolving] = solveQPProblem(robot, tasksTable, q0, q0_d, ...
            rPos, rPos_d, rPos_dd, rOri, rOri_d, rOri_dd, ...
            jointsLBound, jointsUBound, obstacle, simLength, dt)
    qT = [];
    qT_d = [];
    qT_dd = [];
    
    qC = q0;
    qC_d = q0_d;
    
    OPTION.print='';
    
    tSolving = [];
    
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
        d_c = []; % Vector for inequality constraints
        
        while currPriority <= maxPriority
            
            tasksPriority = taskGetSamePriority(currPriority, tasksTable);
            ineqDim = taskGetInequalityDimension(tasksPriority);
            eqDim = taskGetEqualityDimension(tasksPriority);
            
            A = []; % A matrices of the Ob. Fun.
            b = []; % b vectors of the Ob. Fun.
            C = [];
            d = [];
            
            for ii = 1:length(tasksPriority)
                task = tasksPriority(ii);
                if task.equality == 1
                    A = [A; task.A];
                    b = [b; task.b];
                elseif task.equality == 0
                    C = [C; task.A];
                    d = [d; task.b];
                end                
            end
                   
            nVars = robot.n + ineqDim + 1; % Number of primal variables [q1, ..., qn, slack, gamma]
            nBlocks = 1 + 1*(ineqDim~=0) + ~isempty(A_c)*2 + ~isempty(C_c)*1; % Number of Blocks in F
            
            bStruct = [eqDim + ineqDim + 1];
            if (ineqDim~=0)
                bStruct = [bStruct; -ineqDim] ;
            end
            if ~isempty(A_c)
                bStruct = [bStruct; -(size(A_c,1)); -(size(A_c,1))];
            end
            if ~isempty(C_c)
                bStruct = [bStruct; -(size(C_c,1))];
            end
            
            % Create F matrices
            if isempty(A)
                A_comp = [zeros(ineqDim, robot.n),eye(ineqDim)];
            else
                A_comp = blkdiag(A,eye(ineqDim));
            end
            b_comp = [b; zeros(ineqDim,1)];
            g_comp = -2*b_comp'*A_comp;
            
            % Filling matrices related with objective function
            F_matrices = SDPFillOF(A_comp, g_comp, C, d, nVars, robot.n, ineqDim~=0);
            
            % Filling constraints from higher hierarchy tasks
            F_matrices = SDPFillConst(A_c, b_c, C_c, d_c, nVars, robot.n, ineqDim, F_matrices);

            % Solve the problem with SDPA-M
            c_vec = [zeros(nVars-1, 1); 1];
            [objVal, xOpt, X, Y, INFO] = sdpam(nVars, nBlocks, bStruct, c_vec, F_matrices,[],[],[],OPTION);
            
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
        
        % After finishing the task opt. do a last minimization optimization
        nVars = robot.n + 1; % Number of primal variables [q1, ..., qn, slack, gamma]
        nBlocks = 1 + ~isempty(A_c)*2 + ~isempty(C_c)*1; % Number of Blocks in F

        bStruct = [robot.n + 1];
        if ~isempty(A_c)
            bStruct = [bStruct; -(size(A_c,1)); -(size(A_c,1))];
        end
        if ~isempty(C_c)
            bStruct = [bStruct; -(size(C_c,1))];
        end

        % Create F matrices
        A_comp = eye(robot.n);
        b_comp = zeros(robot.n,1);
        g_comp = -2*b_comp'*A_comp;
        
         % Filling matrices related with objective function
        F_matrices = SDPFillOF(A_comp, g_comp, C, d, nVars, robot.n, 0);
        % Filling constraints from higher hierarchy tasks
        F_matrices = SDPFillConst(A_c, b_c, C_c, d_c, nVars, robot.n, 0, F_matrices);

        % Solve the problem with SDPA-M
        c_vec = [zeros(nVars-1, 1); 1];
        [objVal, xOpt, X, Y, INFO] = sdpam(nVars, nBlocks, bStruct, c_vec, F_matrices,[],[],[],OPTION);
        qC_dd = xOpt(1:robot.n);
        % /------Solving Hierarchy problem
        
        % Computing Euler integration
        qC_d_aux = qC_d + qC_dd*dt;
        qC_aux = qC + qC_d*dt + 1/2*qC_dd*dt^2;

        qC_d = qC_d_aux;
        qC = qC_aux;

        % Store current joint acceleration
        qT_dd =  [qT_dd, qC_dd];
        tSolving = [tSolving, toc];       

    end   
end
