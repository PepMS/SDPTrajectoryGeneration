function enabled = taskIsEnabled(taskName,taskTable)
    task = Task;
    for i=1:length(taskTable)
        task = taskTable(i);
        if strcmp(task.name, taskName) && task.enabled
            enabled = true;
            return
        end
    end
    enabled = false;
end