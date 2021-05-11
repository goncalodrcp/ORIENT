%% Trajectory with the Kinova Gen3 - Rotation only
%
% Gonçalo Pereira, nº 81602
% 
% ORIENT
%% Setup
clc
clear;
close all;

addpath('../Utilities')

%Original files from MATLAB blog post
load gen3
gen3.Gravity = [0 0 -9.81];
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

 
n = 3;
maximum = 2^n;
 
result = dec2bin(0:maximum-1,n);
result = double(result) - '0';
result(result==0) = -1;

maxAngle = pi/4;
limitRot = result*maxAngle;

% Euler Angles (ZYX)
% Roll-Pitch-Yaw
rollMaxAngle=maxAngle; %Z-Axis rotation
pitchMaxAngle=pi/8; %Y-Axis rotation
yawMaxAngle=maxAngle; %X-Axis rotation

% Define number of waypoints
numWaypoints = 2;

% Define step
stepYaw = yawMaxAngle/(numWaypoints-1);
stepPitch = pitchMaxAngle/(numWaypoints-1);
stepRoll = rollMaxAngle/(numWaypoints-1);

% Create vector with the waypoints according to each rotation axis
yawAngles = 0:stepYaw:yawMaxAngle;
rollAngles = 0:stepRoll:rollMaxAngle;
pitchAngles = 0:stepPitch:pitchMaxAngle;


% Initialize orientation of each axis
orientations = zeros(3,numWaypoints);

%orientations(1,:) = -result(1,1)*rollAngles;
orientations(2,:) = -result(1,2)*pitchAngles;
orientations(3,:) = result(1,3)*yawAngles;          

%Waypoints - they should remain the same since we're only performing
%rotation
% Positions (X Y Z)
waypoints = repmat(toolPositionHome',1,numWaypoints); 
           
% Array of waypoint times
duration = 5; % Define total duration of the movement (s)
timeStep = duration/(numWaypoints-1);
waypointTimes = 0:timeStep:duration;
% Trajectory sample time
ts = 0.001; % Sampling time of the robot is 1ms
trajTimes = 0:ts:waypointTimes(end);
% Derived end effector positions in joint space trajectory
posJoint = zeros(3,numel(trajTimes));
% Derived end effector orientation in joint space 
eeEuler = zeros(numel(trajTimes),3);
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

%% Trajectory generation and following loop 

%Perform interpolation and solve IK at 1kHz (1ms)
[interpInfo,ikInfo,trajGenTime] = rotationInterpAndIK(numWaypoints,orientations, ... 
                                   toolPositionHome,waypointTimes,ts,eeName,ik,ikWeights,ikInitGuess);
% 20ms                              
[interpInfo_mid,ikInfo_mid,trajGenTime_mid] = rotationInterpAndIK(numWaypoints,orientations, ... 
                                   toolPositionHome,waypointTimes,0.02,eeName,ik,ikWeights,ikInitGuess);                               
% 50ms            
[interpInfo_low,ikInfo_low,trajGenTime_low] = rotationInterpAndIK(numWaypoints,orientations, ... 
                                   toolPositionHome,waypointTimes,0.05,eeName,ik,ikWeights,ikInitGuess);                               


disp(['Trajectory generation time (1ms) : ' num2str(trajGenTime) ' s']);
disp(['Trajectory generation time (20ms) : ' num2str(trajGenTime_mid) ' s']);
disp(['Trajectory generation time (50ms) : ' num2str(trajGenTime_low) ' s']);

%% Plot different ik results

for idx = 1:numJoints
    figure, hold on;
    plot(ikInfo.time,ikInfo.jointAngles(:,idx));
    plot(ikInfo_mid.time,ikInfo_mid.jointAngles(:,idx));
    plot(ikInfo_low.time,ikInfo_low.jointAngles(:,idx));
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' Trajectory']); 
    xlabel('Time [s]');
    ylabel('Joint Angle [rad]');
    legend('IK(1ms)','IK(20ms)','IK(50ms)');
    figure, hold on
    plot(ikInfo.time,ikInfo.jointVel(:,idx));
    plot(ikInfo_mid.time,ikInfo_mid.jointVel(:,idx));
    plot(ikInfo_low.time,ikInfo_low.jointVel(:,idx));
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' velocity']); 
    xlabel('Time [s]');
    ylabel('Angular velocity [rad/s]');
    legend('IK(1ms)','IK(20ms)','IK(50ms)');
    figure, hold on
    plot(ikInfo.time,ikInfo.jointAcc(:,idx));
    plot(ikInfo_mid.time,ikInfo_mid.jointAcc(:,idx));
    plot(ikInfo_low.time,ikInfo_low.jointAcc(:,idx));
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' acceleration']); 
    xlabel('Time [s]');
    ylabel('Angular acceleration [rad/s^2]');
    legend('IK(1ms)','IK(20ms)','IK(50ms)');
end

%% Interpolate all trajectories to 1ms

timeInterp = 0:ts:trajTimes(end);
jointAngles_mid = interp1(ikInfo_mid.time,ikInfo_mid.jointAngles,timeInterp,'makima');
jointAngles_low = interp1(ikInfo_low.time,ikInfo_low.jointAngles,timeInterp,'makima');

jointVel_mid = interp1(ikInfo_mid.time,ikInfo_mid.jointVel,timeInterp,'makima');
jointVel_low = interp1(ikInfo_low.time,ikInfo_low.jointVel,timeInterp,'makima');

jointAcc_mid = interp1(ikInfo_mid.time,ikInfo_mid.jointAcc,timeInterp,'makima');
jointAcc_low = interp1(ikInfo_low.time,ikInfo_low.jointAcc,timeInterp,'makima');

%% Compare interpolations to original IK

for idx = 1:numJoints
    figure, hold on;
    plot(ikInfo.time,ikInfo.jointAngles(:,idx));
    plot(timeInterp,jointAngles_mid(:,idx));
    plot(timeInterp,jointAngles_low(:,idx));
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' Trajectory']); 
    xlabel('Time [s]');
    ylabel('Joint Angle [rad]');
    legend('IK(1ms)','IK(20ms) interpolated','IK(50ms) interpolated');
    figure, hold on
    plot(ikInfo.time,ikInfo.jointVel(:,idx));
    plot(timeInterp,jointVel_mid(:,idx));
    plot(timeInterp,jointVel_low(:,idx));
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' velocity']); 
    xlabel('Time [s]');
    ylabel('Angular velocity [rad/s]');
    legend('IK(1ms)','IK(20ms) interpolated','IK(50ms) interpolated');
    figure, hold on
    plot(ikInfo.time,ikInfo.jointAcc(:,idx));
    plot(timeInterp,jointAcc_mid(:,idx));
    plot(timeInterp,jointAcc_low(:,idx));
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' acceleration']); 
    xlabel('Time [s]');
    ylabel('Angular acceleration [rad/s^2]');
    legend('IK(1ms)','IK(20ms) interpolated','IK(50ms) interpolated');
end

save('trajData.mat','trajTimes','ikInfo','ikInfo_low','ikInfo_mid','interpInfo','interpInfo_low','interpInfo_mid', ...,
     'jointAcc_low','jointAcc_mid','jointVel_low','jointVel_mid','jointAngles_low','jointAngles_mid');
     