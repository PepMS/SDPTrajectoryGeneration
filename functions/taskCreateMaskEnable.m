function maskEnable = taskCreateMaskEnable(taskTable)
    maskEnable = 0;
    task = Task;
    for i=1:length(taskTable)
        task = taskTable(i);
        maskEnable = maskEnable + bitsll(task.enabled, i-1);
    end
    maskEnable = de2bi(maskEnable);
end
