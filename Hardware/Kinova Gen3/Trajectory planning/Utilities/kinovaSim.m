%% KINOVA Gen3 Manipulator simulation

clear;
close all;

% Load robot
robot = loadrobot('kinovaGen3','DataFormat','row','Gravity',[0 0 -9.81]);

% Set robot to home configuration
currentRobotJConfig = homeConfiguration(robot);

numJoints = numel(currentRobotJConfig); %Number of joints
endEffector = "EndEffector_Link"; %

timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s

jointInit = currentRobotJConfig;
taskInit = getTransform(robot,jointInit,endEffector);

taskFinal = trvec2tform([0.4,0,0.6])*axang2tform([0 1 0 pi]);

distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));

initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];

[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 

tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robot,'EndEffectorName','EndEffector_Link');

tsMotionModel.Kp(1:3,1:3) = 0;
tsMotionModel.Kd(1:3,1:3) = 0;

q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));

[tTask,stateTask] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; qd0]);

