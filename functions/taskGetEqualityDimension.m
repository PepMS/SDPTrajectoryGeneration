% Returns the number of equality tasks of a given table of tasks
function eDim = taskGetEqualityDimension(tasksTable)
    eDim = 0;
    for i = 1:length(tasksTable)
        task = tasksTable(i);
        if task.equality == 1
            eDim = eDim + length(task.b);
        else
        end
    end
end