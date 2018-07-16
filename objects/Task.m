classdef Task
   properties
      name          % Name of the task
      enabled       % 1 -> Task enabled; 0 -> Task disabled
      priority      % Set the priority. 1 -> Highest priority
      A             % A*x <= b
      b
      robot         % Serial Link object associated to the task
      equality      % 1 -> Equality constraint; 0 -> Inequality constraint 
      plot          % 1 -> Plot the task at the end; 0 -> Do not plot
      avoidJoint    % In case of being a task for obstacle avoidance, 
                    % this specifies which joint must avoid the obstacle
        
   end
   methods
   
   end
end