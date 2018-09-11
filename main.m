%% For help type "rtbdemo"
clearvars; clc; close all;

mtTitle = 'Hierarchical Trajectory Gen.: ';

disp(strcat(mtTitle,'Loading libraries'))

addpath('objects', 'functions');
addpath('objects', 'functions');
addpath(genpath('~/sdpa/share/sdpa/mex'));
%% Simulation Settings and Parameters - EDIT

% Robot
nLinks = 4; % Links of the planar robot
dLinks = [0.5, 0.433, 0.35, 0.22]; % Lengths of the robot's links

% Joints Limits
jointsUppBound = [1000; 180; 180; 1000]; % Degrees, 1000 for disabled limit
jointsLowBound = [-1000; 5; 5; -1000]; % Degrees, -1000 for disabled limit

% Position Reference
rPos_Ini = [1, 0.8]; % End-effector initial position
rPos_End = [1, -0.5]; % End-effector final position

% Orientation Angle Reference
rOriAngle = 90;
d2r = pi/180; r2d = d2r^-1;

vMax = 0.05; % End-effector max. velocity
aMax = 0.005; % End-effector max. acceleration
dt = 0.1; % Time step for trajecory generation

% Obstacle
obs = Obstacle;
obs.radius = 0.30; % radius of the obstacle modeled as a cylinder
obs.center = [1.1; 0.1]; % center of the obstacle

% Animation & Plots
simAnimation = 1;
plotsEnabled = 1;
plotTime     = 1;

%% Config Errors - DO NOT EDIT
if length(dLinks) ~= nLinks
    error(strcat('Dimension of the vector indicating lengths of the', ...
    'links and "nLinks" DO NOT coincide'));
end 

if length(obs.center) ~= 2
    error('Center of the obstacle defined is not 2-D')
end

%% Creating the robot - DO NOT EDIT

disp(strcat(mtTitle, ' Creating Robot'))
robot = robotCreate(dLinks);
disp(strcat(mtTitle, ' Robot Created'))

%% Task Creation - EDIT
tasksExisting = [];
task = Task;

% In case to add another task copy-paste the code between "-" 
% and fill in accordingly. Also, remember to include the proper method for 
% the computation of A and b matrices in the 'Task' object

% IMPORTANT: Do not change the name of the tasks as it is used in the 
% function 'tasksInitialized()' to distinguish among tasks  

% <Copy-Paste>------------------
task.name = 'Position_Tracking'; % Do not change this name
task.enabled = 1;
task.priority = 3;
task.A = [];
task.b = [];
task.equality = 1;
task.plot = 1;
task.robot = robot;
tasksExisting = taskAdd(tasksExisting, task);
% <\Copy-Paste>-----------------

task.name = 'Orientation_Tracking';
task.enabled = 1;
task.priority = 2;
task.A = [];
task.b = [];
task.equality = 1;
task.plot = 1;
task.robot = robot;
tasksExisting = taskAdd(tasksExisting, task);

task.name = 'Joint_Limits';
task.enabled = 1;
task.priority = 1;
task.A = [];
task.b = [];
task.equality = 0;
task.plot = 1;
task.robot = robot;
tasksExisting = taskAdd(tasksExisting, task);

task.name = 'Obstacle_Avoidance'; % Do not change this name
task.enabled = 1;
task.priority = 1;
task.A = [];
task.b = [];
task.equality = 0;
task.plot = 1;
task.robot = robot;
% Avoidance Joints
%  = 1 -> EndEffector
%  = 2 -> Joint after the EndEffector
%  = ...
avoidanceJoints = 4;
tasksExisting = taskAdd(tasksExisting, task, 'avoidanceJoints', avoidanceJoints);

task.name = 'Manipulability';
task.enabled = 0;
task.priority = 2;
task.A = [];
task.b = [];
task.equality = 1;
task.plot = 0;
task.robot = robot;
tasksExisting = taskAdd(tasksExisting, task);

%% Task numbering - DO NOT EDIT

taskMaskEnable = taskCreateMaskEnable(tasksExisting);

%% Creating Objects & Scenario - DO NOT EDIT

if taskIsEnabled('Position_Tracking', tasksExisting)
    disp(strcat(mtTitle, ' Creating Trajectory'))
    [rPos, rPos_d, rPos_dd, rPos_t] = trajectoryGenerator(rPos_Ini, rPos_End, vMax, aMax, dt);
    disp(strcat(mtTitle, ' Trajectory Created'))
else
    rPos = [];
end

if taskIsEnabled('Orientation_Tracking', tasksExisting)
    disp(strcat(mtTitle, ' Creating Trajectory'))
    rOri = cosd(rOriAngle)*ones(length(rPos), 1);
    rOri_d = zeros(length(rPos), 1);
    rOri_dd = zeros(length(rPos), 1);
    disp(strcat(mtTitle, ' Trajectory Created'))
else
    rOri = zeros(length(rPos), 1);
    rOri_d = zeros(length(rPos), 1);
    rOri_dd = zeros(length(rPos), 1);
end

if taskIsEnabled('Joint_Limits', tasksExisting)
    if (robot.n ~= length(jointsUppBound)) || ...
            (robot.n ~= length(jointsLowBound))
        error(strcat('Number of links and limit vectors are different'))
    end
end

if taskIsEnabled('Obstacle_Avoidance', tasksExisting)
else
    obs = [];
end

disp(strcat(mtTitle, ' ScenarioCreated'))

%% Trajectory Generation

tasksOrdered = taskOrder(tasksExisting, taskMaskEnable);

simLength = length(rPos);
angle0 = rOriAngle*d2r;
T_ini = transl([rPos_Ini, 0]) * rpy2tr(0, 0, angle0);
q0 = robot.ikunc(T_ini);

disp(strcat(mtTitle, ' Solving Problem...'))
  
[qT, qT_d, qT_dd, solTime] = solveSDPProblem(robot, tasksOrdered, q0', zeros(robot.n, 1), ...
                             rPos', rPos_d', rPos_dd', ...
                             rOri, rOri_d, rOri_dd, ...
                             jointsLowBound, jointsUppBound,...
                             obs, simLength, dt);
%% Plotting results
disp(strcat(mtTitle, ' Plotting Results...'))

% Defining nice colors
colors = {[0,      0     , 1], 
          [0,      0.5   , 0], 
          [0,      0.4470, 0.7410],	          	
          [0.8500, 0.3250, 0.0980],	         
          [0.9290, 0.6940, 0.1250],	          	
          [0.4940, 0.1840, 0.5560],	          	
          [0.4660, 0.6740, 0.1880],	          	
          [0.3010, 0.7450, 0.9330],	          	
          [0.6350, 0.0780, 0.1840]};

if plotsEnabled
    taskPlot(tasksOrdered, solTime, plotTime, robot, qT, rPos', rOri, ...
             jointsLowBound, jointsUppBound, obs, avoidanceJoints, colors);
end

if simAnimation
    if ~exist('figRobot' , 'var')
       figRobot = figure;
       scenarioPlot(robot, rPos, obs);
       figure(figRobot);
    end
    robot.plot(qT', 'fps', 64)
end

