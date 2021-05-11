%% Setup
clc
clear;
close all;

addpath('../utilities')

%Original files from MATLAB blog post
load gen3
%load gen3positions %1st configuration 
load robotConfig2 %2nd configuration
%TO DO: verify these configurations 

%Load the gripper information
eeName = 'Gripper';
numJoints = numel(gen3.homeConfiguration);

figure;
show(gen3,jointAnglesHome');
T_home = getTransform(gen3,jointAnglesHome',eeName); %Get transformation matrix from base to gripper in home configuration

%% Define waypoints

% Euler Angles (ZYX)
% Yaw-Pitch-Roll
rollMaxAngle=pi/6; %X-Axis rotation
pitchMaxAngle=pi/6; %Y-Axis rotation
yawMaxAngle=pi/6; %Z-Axis rotation
%maxAngle = pi/3;

% Define number of waypoints
numWaypoints = 5;

% Define step
stepRoll = rollMaxAngle/(numWaypoints-1);
stepPitch = pitchMaxAngle/(numWaypoints-1);

% Create vector with the waypoints according to each rotation axis
rollAngles = 0:stepRoll:rollMaxAngle;
pitchAngles = 0:stepPitch:pitchMaxAngle;

% Initialize orientation of each axis
orientations = zeros(3,numWaypoints);

orientations(2,:) = pitchAngles;
orientations(3,:) = rollAngles;
     
% orientations = [0     0    0;
%                 0     0    pi/16;
%                 0     0    pi/8;
%                 0     0    3*pi/16;
%                 0     0    pi/4]';            

%Waypoints - they should remain the same since we're only performing
%rotation
% Positions (X Y Z)
waypoints = repmat(toolPositionHome',1,numWaypoints); 
           
% Array of waypoint times
duration = 1.6; % Define total duration of the movement (s)
timeStep = duration/(numWaypoints-1);
waypointTimes = 0:timeStep:duration;
% Trajectory sample time
ts = 0.01; % Sampling time of the robot is 1ms
trajTimes = 0:ts:waypointTimes(end);
% Derived end effector positions in joint space trajectory
posJoint = zeros(3,numel(trajTimes)); 
% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;

%% Show waypoints and robot in home configuration

% Define IK
ik = inverseKinematics('RigidBodyTree',gen3);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = jointAnglesHome';
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

% Set up plot
plotMode = 2; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(gen3,gen3.homeConfiguration,'Frames','off','PreservePlot',false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);


%% Solve IK for waypoints only (Joint space) - Just a test

disp('Running joint space trajectory generation and evaluation...')
tic;
% Solve IK for all waypoints
jointWaypoints = zeros(numJoints,numWaypoints);

%Different configurations in quaternions
for i = 1:numWaypoints
    Q(i,:) = eul2quat(orientations(:,i)'); %Get each orientation quaternion
    tgtPose = trvec2tform(toolPositionHome) * quat2tform(Q(i,:)); %Get pose of gripper
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    cfgDiff = config - ikInitGuess;
    jointWaypoints(:,i) = config';    
end

% Trajectory Generation
[qJoint,qdJoint,qddJoint] = trapveltraj(jointWaypoints,numel(trajTimes), ...
                    'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
                    'EndTime',repmat(diff(waypointTimes),[numJoints 1]));

% Trajectory evaluation (only needed to find end effector position)
for idx = 1:numel(trajTimes)  
    eeTform = getTransform(gen3,qJoint(:,idx)',eeName); 
    posJoint(:,idx) = tform2trvec(eeTform)';
    show(gen3,config,'Frames','off','PreservePlot',false);
end

jointTime = toc;
disp(['Joint space trajectory time : ' num2str(jointTime) ' s']);

% To visualize the trajectory
plotTrajectory(trajTimes,qJoint,qdJoint,qddJoint,'Names',"Joint " + string(1:numJoints),'WaypointTimes',waypointTimes)
