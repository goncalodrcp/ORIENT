%% Trajectory with the Kinova Gen3 - Rotation only
%
% Gonçalo Pereira, nº 81602
% 
% ORIENT
%% Setup
clc
clear;
close all;%

addpath('../Utilities')
addpath('D:\IST\ORIENT_repos\ORIENT\MATLAB (main)\Helpers\ProcessRosbag');

%Load 3D Model of the robot with custom gripper
loadSTL;
%load gen3positions %1st configuration 
load robotConfig2 %2nd configuration
%TO DO: verify these configurations 

%Joint limits (speed and acceleration) - Check datasheet for more info
jointSpeedLimit = 50*(pi/180); %rad/s
jointAccLimit = 57.3*(pi/180); %rad/s^2

%Load the gripper information
eeName = 'Gripper';
numJoints = numel(gen3.homeConfiguration);

figure;
show(gen3,jointAnglesHome');
T_home = getTransform(gen3,jointAnglesHome',eeName); %Get transformation matrix from base to gripper in home configuration

%% Define waypoints

%Define rotation waypoints
axis = 'Z';
amplitude = 20; % in degrees
waypoints = [0 amplitude -amplitude 0 -amplitude amplitude]*pi/180;
waypointTimes = [0 4 8 12 16 20];
ts = 0.05; %Sampling time (s)
numWaypoints = length(waypoints);
[waypointTrajectory] = defineWaypoints(waypoints,axis,toolPositionHome',waypointTimes,ts);


%% Define Inverse Kinematics

% Define IK
ik = inverseKinematics('RigidBodyTree',gen3);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = jointAnglesHome';
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

% Set up plot
% plotMode = 2; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
% show(gen3,gen3.homeConfiguration,'Frames','off','PreservePlot',false);
% xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
% hold on
% if plotMode == 1
%     hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
% end
% plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% Trajectory generation and following loop 

%Perform interpolation and solve IK at 50ms
[interpInfo,ikInfo,trajGenTime] = rotationInterpAndIK(numWaypoints,waypointTrajectory.orientations, ... 
                                   toolPositionHome,waypointTimes,ts,eeName,ik,ikWeights,ikInitGuess);
                                                              
%Retrieve information
rot = interpInfo.Rot;
angv = interpInfo.AngVel;
angacc = interpInfo.AngAcc;
trajTimes = waypointTrajectory.trajtimes;

disp(['Trajectory generation time : ' num2str(trajGenTime) ' s']);

%% Plot trajectory - Interpolation 

% Get rotational trajectory in euler angles
eulerAngles = quat2eul(rot(:,1:size(rot,2))');

% Plot trajectories generated from quaternion interpolation
plotInterpolation(trajTimes,waypointTimes,eulerAngles,angv,angacc);

MaxVelocity = max(max(angv,[],2))*180/pi;


%% Plot trajectory - IK

% Plot joint trajectory from Inverse Kinematics
helperPlotIK(ikInfo,ts,waypointTimes,jointSpeedLimit,jointAccLimit)

%% Recompute trajectory for each joint

trajectoryToSend = compute1kHzTrajectory(ikInfo,waypointTimes,trajTimes);

%% Replay trajectory that is going to be sent

trajectoryToSend.angles = trajectoryToSend.angles';
trajectoryToSend.velocity = trajectoryToSend.velocity';
trajectoryToSend.acceleration = trajectoryToSend.acceleration';
replayTrajectoryKinova(gen3,trajectoryToSend);
plotSentTrajectory(gen3,waypointTimes,trajectoryToSend);

%% Save trajectory

%path = '/media/goncalopereira/DATA/IST/ORIENT_repos/Tests/ThesisSW/Data collected/Experiments_24_07/Sent';
% path = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Sent\Z-Axis';
% 
% fileName = '\Z_A20_v13.mat';
% 
% savefile = strcat(path,fileName);
% 
% save(savefile,'trajectoryToSend','trajTimes','ikInfo','interpInfo');

