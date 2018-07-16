function taskTable_ = taskAdd(taskTable, task, varargin)
    defaultAvoidanceJoints = 1;
    
    p = inputParser;
    
    validScalarPosNum = @(x) isnumeric(x) && isscalar(x) && (x > 0);
    addRequired(p, 'taskTable');
    addRequired(p, 'task');
    addOptional(p, 'avoidanceJoints', defaultAvoidanceJoints, validScalarPosNum);
    parse(p, taskTable, task, varargin{:});
    
    aJoints = p.Results.avoidanceJoints;
    
    if ~ strcmp(task.name, 'Obstacle_Avoidance')
        taskTable_ = [taskTable, task];
    elseif strcmp(task.name, 'Obstacle_Avoidance') 
        if aJoints > task.robot.n
            error('Joints specified in variable "avoidanceJoints" is greater than the number of joints');
        else
            j = task.robot.n;
            taskTable_ = taskTable;
            for i = 1:aJoints
                task.avoidJoint = j;
                j = j - 1;
                taskTable_ = [taskTable_, task];
            end
        end
    end
end
