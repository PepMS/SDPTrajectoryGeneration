% Returns the number of inequality task of a given table of tasks
function iDim = taskGetInequalityDimension(tasksTable)
    iDim = 0;
    for i = 1:length(tasksTable)
        task = tasksTable(i);
        if task.equality == 0
            iDim = iDim + length(task.b);
        else
        end
    end
end
