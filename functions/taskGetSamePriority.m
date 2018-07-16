% This function goes through all the table and returns a table of tasks
% with a priority equal to the one specified by the variable 'priority'
function tasksPriority = taskGetSamePriority(priority, tasksTable)
    tasksPriority = [];
    for i = 1:length(tasksTable)
        task = tasksTable(i);
        if task.priority == priority
            tasksPriority = [tasksPriority, task];
        end
    end
end