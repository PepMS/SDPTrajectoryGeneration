function [A, b, C, d, C_disc, d_disc, eqDim, ineqDim] =  SDPLoadTasksInMatrices(currPriority, tasksTable, activeSet, qC_dd)
        
    tasksPriority = taskGetSamePriority(currPriority, tasksTable);
    A = []; % A matrices of the Ob. Fun.
    b = []; % b vectors of the Ob. Fun.
    C = [];
    d = [];
    C_disc = [];
    d_disc = [];
    
    
    if ~activeSet
        ineqDim = taskGetInequalityDimension(tasksPriority);
        eqDim = taskGetEqualityDimension(tasksPriority);

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
        
    elseif activeSet      
        for ii = 1:length(tasksPriority)
            task = tasksPriority(ii);
            if task.equality == 1
                A = [A; task.A];
                b = [b; task.b];
            elseif task.equality == 0
                actMask = (task.A*qC_dd-task.b > -0.5);
                C_disc = [C_disc; task.A(actMask==0,:)];
                d_disc = [d_disc; task.b(actMask==0)];
                C = [C; task.A(actMask==1,:)];
                d = [d; task.b(actMask==1)];
            end                
        end
        ineqDim = length(d);
        eqDim = length(b);
        
    end
end
