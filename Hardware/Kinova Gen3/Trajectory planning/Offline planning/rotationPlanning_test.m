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

n = 3;
max = 2^n;
 
result = dec2bin(0:max-1,n);
result = double(result) - '0';
result(result==0) = -1;

maxAngle = pi/4;
limitRot = result*maxAngle;

% Euler Angles (ZYX)
% Roll-Pitch-Yaw
rollMaxAngle=maxAngle; %Z-Axis rotation
pitchMaxAngle=maxAngle; %Y-Axis rotation
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

orWaypoints = [0 pi/6 -pi/6 pi/6 0];
numWaypoints = length(orWaypoints);
% Initialize orientation of each axis
orientations = zeros(3,numWaypoints);
orientations(1,:) = orWaypoints;
 
% orientations(1,:) = result(1,1)*rollAngles;
% orientations(2,:) = result(1,2)*pitchAngles;
% orientations(3,:) = result(1,3)*yawAngles;
     
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
duration = 16; % Define total duration of the movement (s)
%timeStep = duration/(numWaypoints-1);
%waypointTimes = 0:timeStep:duration;
%waypointTimes = linspace(0,duration,numWaypoints);
waypointTimes = [0 4 8 12 16];
% Trajectory sample time
ts = 0.05; % Sampling time of the robot is 1ms
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
                                                              
%Retrieve information
jointAnglesIK = ikInfo.jointAngles;
timesIK = ikInfo.time;
rot = interpInfo.Rot;
angv = interpInfo.AngVel;
angacc = interpInfo.AngAcc;

disp(['Trajectory generation time : ' num2str(trajGenTime) ' s']);


%% Plot trajectories - Generated and IK

% Get rotational trajectory in euler angles
eulerAngles = quat2eul(rot(:,1:size(rot,2))');

% Plot trajectories generated from quaternion interpolation
plotInterpolation(trajTimes,waypointTimes,eulerAngles,angv,angacc);

% Plot Inverse Kinematics joint angles
% Compare joint angles
% Plot each joint trajectory
angularVelIK=diff(jointAnglesIK)/ts;
angularVelIK(1,:) = 0;
angularVelIK(end+1,:) = 0;
     
angularAccIK=diff(angularVelIK)/ts;
angularAccIK(1,:) = 0;
angularAccIK(end+1,:) = 0;

anglePlot = figure;
velPlot = figure;
accPlot = figure;

for idx=1:numJoints
    
    % Angle plot
    figure(anglePlot), hold on;
    plot(timesIK,jointAnglesIK(:,idx));
    for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
    end
     title('Inverse Kinematics results - angle'); 
     xlabel('Time [s]');
     ylabel('Joint Angle [rad]');
     
     %Velocity plot
     figure(velPlot), hold on;
     plot(timesIK,angularVelIK(:,idx));
     yline(jointSpeedLimit,'r','Max. Speed'); %Speed limit
     for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
     end
     title('Inverse Kinematics results - velocity'); 
     xlabel('Time [s]');
     ylabel('Angular velocity [rad/s]');
     
     % Acceleration plot
     figure(accPlot), hold on;
     plot(timesIK,angularAccIK(:,idx));
     yline(jointAccLimit,'r','Max. Acceleration'); %Acceleration limit
     for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
     end
     title('Inverse Kinematics results - acceleration'); 
     xlabel('Time [s]');
     ylabel('Angular acceleration [rad/s^2]');
          
end


%% Replay trajectory

% Create figure and hold it
figure
set(gcf,'Visible','on');
show(gen3, jointAnglesHome');
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
 % Loop through values at specified interval and update figure
 count=1;
 for i = 1:length(trajTimes)
    poseEE = getTransform(gen3,jointAnglesIK(i,:),eeName);
    plotTransforms(tform2trvec(poseEE),tform2quat(poseEE),'FrameSize',0.05);
   % Display manipulator model
   show(gen3, jointAnglesIK(i,:), 'Frames', 'off', 'PreservePlot', false);
   title(['Trajectory at t = ' num2str(trajTimes(i))]);
   % Update figure
   drawnow
   frames(count)=getframe(gcf); %store frames for a video
   count = count + 1;
 end

 %%
 
% Record video animation
%recordVideoAnimation('test.avi',frames);


%% Recompute trajectory for each joint
%close all;
%Joint angles home? or first guess from IK
jointWaypoints = [];
for i=1:length(waypointTimes)
    jointWaypoint = jointAnglesIK(trajTimes == waypointTimes(i),:);
    jointWaypoints = [jointWaypoints jointWaypoint'];
end

%jointWaypoints = [jointAnglesIK(1,:)' jointAnglesIK(end,:)'];
sampleTime = 0.001;
numSamples = waypointTimes(end)/sampleTime + 1; 
[qJoint,qdJoint,qddJoint] = trapveltraj(jointWaypoints,numSamples, ... 
                    'EndTime',repmat(diff(waypointTimes),[numJoints 1]));
                
%[qJoint,qdJoint,qddJoint] = trapveltraj(jointWaypoints,numel(trajTimes), ... 
%                    'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
%                    'EndTime',repmat(diff(waypointTimes),[numJoints 1]));

trajTimes = linspace(0,waypointTimes(end),numSamples);                                             
eeTform = cell(1,numel(trajTimes));
for i=1:numel(trajTimes)
     eeTform{i} = getTransform(gen3,qJoint(:,i)',eeName);
     eeEuler(i,:) = tform2eul(eeTform{i});
     posJoint(:,i) = tform2trvec(eeTform{i})';
end

for idx = 1:numJoints
    figure, hold on;
    plot(timesIK,jointAnglesIK(:,idx));
    plot(trajTimes,qJoint(idx,:));
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' Trajectory']); 
    xlabel('Time [s]');
    ylabel('Joint Angle [rad]');
    legend('Inverse Kinematics results','Corrected trajectory (trapezoidal)');
    figure, hold on
    plot(timesIK,angularVelIK(:,idx));
    plot(trajTimes,qdJoint(idx,:));
    yline(jointSpeedLimit,'r','Max. Speed')
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' velocity']); 
    xlabel('Time [s]');
    ylabel('Angular velocity [rad/s]');
    legend('Inverse Kinematics results','Corrected trajectory (trapezoidal)');
    figure, hold on
    plot(timesIK,angularAccIK(:,idx));
    plot(trajTimes,qddJoint(idx,:));
    yline(jointSpeedLimit,'r','Max. Acceleration')
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' acceleration']); 
    xlabel('Time [s]');
    ylabel('Angular acceleration [rad/s^2]');
    legend('Inverse Kinematics results','Corrected trajectory (trapezoidal)'); 
end

save('Results\Experiments_16_07\trajZAxis_16_07_fast.mat','trajTimes','ikInfo','interpInfo');

%% Replay recomputed trajectory

% % Create figure and hold it
% figure
% set(gcf,'Visible','on');
% show(gen3, jointAnglesHome');
% xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
% hold on
%  % Loop through values at specified interval and update figure
%  count=1;
%  for i = 1:10:length(trajTimes)
%    plotTransforms(tform2trvec(eeTform{i}),tform2quat(eeTform{i}),'FrameSize',0.05);
%    % Display manipulator model
%    show(gen3, qJoint(:,i)', 'Frames', 'off', 'PreservePlot', false);
%    title(['Trajectory at t = ' num2str(trajTimes(i))]);
%    % Update figure
%    drawnow
%    frames(count)=getframe(gcf); %store frames for a video
%    count = count + 1;
%  end


                