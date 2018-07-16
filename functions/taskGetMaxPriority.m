% This function gets the maximum number in the priority field of the tasks
% inside the "taskTable"
function maxPriority = taskGetMaxPriority(taskTable)
    maxPriority = taskTable(end).priority;
end