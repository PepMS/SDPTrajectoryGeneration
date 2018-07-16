% This function orders only the enabled tasks 
% according to their priority

% 1 --> HIGHEST Priority

function tasksOrdered = taskOrder(tasksTable, taskMaskEnable)
    task1 = Task;
    task2 = Task;
    
    tasksOrdered = tasksTable(1, taskMaskEnable == 1);
        
    i = 2;
    while i <= length(tasksOrdered)  
        j = i;
        task1 = tasksOrdered(j-1);
        task2 = tasksOrdered(j);
        while j > 1 && task1.priority > task2.priority
            tasksOrdered(j-1) = task2; 
            tasksOrdered(j) = task1;
            j = j - 1;
            if (j <= 1)
                break
            end 
            task1 = tasksOrdered(j-1);
            task2 = tasksOrdered(j);
        end
        i = i + 1;
    end        
end
