function [A, b, C, d, C_disc, d_disc, eqDim, ineqDim] =  SDPLoadTasksInMatrices(currPriority, tasksTable, qC_dd)
        
    tasksPriority = taskGetSamePriority(currPriority, tasksTable);
    A = []; % A matrices of the Ob. Fun.
    b = []; % b vectors of the Ob. Fun.
    C = [];
    d = [];
    C_disc = [];
    d_disc = [];

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
        

end
