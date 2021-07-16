%% Trajectory with the Kinova Gen3 - Rotation only
%
% Gonçalo Pereira, nº 81602
%
% For more information about MATLAB Kortex go to:
% https://github.com/Kinovarobotics/matlab_kortex
% 
% MATLAB examples:
% https://www.mathworks.com/help/supportpkg/robotmanipulator/examples.html?category=get-started&s_tid=CRUX_topnav
% 
% ORIENT
%% Guidelines 
%  
% 1 - Execute script section by section
% 2 - Keep the E-STOP button close to you to stop any unnecessary movements of
% the robot
%
%% Load data from inverse kinematics solved offline
%Later on, switch to function call

clc
clear;
close all;

%%CHANGES 16/07
load('trajXAxis_16_07_slow.mat');
load('timeInfo_slow.mat');
load('path.mat')
addpath(path);
% Trajectory sample time
% ts = 0.05; % Sampling time of the robot is 1ms
% trajTimes = 0:ts:waypointTimes(end);
% numJoints = 7;
%%CHANGES 16/07

%ikInfo = ikInfo_low;
%load('trajData.mat');
%load('trajData_1stRosbagTest.mat','ikInfo','trajTimes')

%Load trajectory at 50ms 
jointAngles= ikInfo.jointAngles;
jointVel = ikInfo.jointVel;
jointAcc = ikInfo.jointAcc;
t = ikInfo.time;
dt = t(2)-t(1);
ts = 0.001;

%Initial joint position (Home) in degrees
jointStart = jointAngles(1,:)*180/pi;
jointStart = wrapTo360(jointStart);
jointStart = round(jointStart);

%Final joint position (After simple motion) in degrees
jointEnd = jointAngles(end,:)*180/pi;
jointEnd = wrapTo360(jointEnd);
jointEnd = round(jointEnd);


% Perform interpolation from 50ms to 1ms
jointAngles = jointAngles*180/pi;
vel = diff(jointAngles)/dt;
vel(1,:) = 0;
vel(end+1,:) = 0;
acc = diff(vel)/dt;
acc(1,:) = 0;
acc(end+1,:) = 0;

timestamp = 0:ts:trajTimes(end);
trajangles = interp1(t,jointAngles,timestamp);
trajvel = interp1(t,vel,timestamp);
trajacc = interp1(t,acc,timestamp);

% jointWaypoints = [];
% for i=1:length(waypointTimes)
%     jointWaypoint = ikInfo.jointAngles(trajTimes == waypointTimes(i),:);
%     jointWaypoints = [jointWaypoints jointWaypoint'];
% end
% 
% sampleTime = 0.001;
% numSamples = waypointTimes(end)/sampleTime + 1; 
% [trajangles,trajvel,trajacc] = trapveltraj(jointWaypoints,numSamples, ... 
%                     'EndTime',repmat(diff(waypointTimes),[numJoints 1]));
%                 
% timestamp = linspace(0,waypointTimes(end),numSamples); 


%% Connection verification
% https://www.mathworks.com/help/supportpkg/robotmanipulator/ug/connect-to-gen3.html

%Substitute for the real robot IP
  IP = '10.0.3.26';
  system(['ping ' IP],'-echo');

%% Create API and connect to robot

Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
gen3Kinova = kortex();
gen3Kinova.ip_address = '10.0.3.26';
%gen3Kinova.user = 'admin';
%gen3Kinova.password = 'admin';
isOk = gen3Kinova.CreateRobotApisWrapper();
if isOk
   disp('You are connected to the robot!'); 
else
   error('Failed to establish a valid connection!'); 
end

[isOk,baseFb, actuatorFb, interconnectFb] = gen3Kinova.SendRefreshFeedback();

if isOk
     disp('Base feedback');
     disp(baseFb);
     disp('Actuator feedback');
     disp(actuatorFb);
     disp('Gripper feedback');
     disp(interconnectFb);
else
    error('Failed to acquire sensor data.'); 
end

[isOk,SensoSettings] = gen3Kinova.SendRefreshFeedback();
if isOk
     disp('Sensor settings collected');
else
    error('Failed to acquire sensor data.'); 
end

%% Send robot to retract

%Initial joint position (Home) in degrees
%jointAnglesRetract = jointAnglesRetract(1,:)*180/pi;
%jointAnglesRetract = wrapTo360(jointAnglesRetract);

%% Send starting point to robot


jointCmd = wrapTo360(trajangles(1,:));
constraintType = int32(0);
speed = 0;
duration = 0;
 
isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
if isOk
    disp('success');
else
    disp('SendJointAngles cmd error');
    return;
end

% Check if the robot has reached the starting position
while 1
    [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
    if isOk
        if max(abs(wrapTo360(trajangles(1,:))-actuatorFb.position)) < 0.2
            disp('Starting point reached.')
            break;
        end 
    else
        error('SendRefreshFeedback error')
    end
end

%% Send pre computed trajectory

isOk = gen3Kinova.SendPreComputedTrajectory(trajangles.', trajvel.', trajacc.', timestamp, size(timestamp,2));
if isOk
    disp('SendPreComputedTrajectory success');
else
    disp('SendPreComputedTrajectory command error');
end

% Check if the robot has reached the starting position
i=1;
while 1
    [isOk,~, actuatorFb(i), ~] = gen3Kinova.SendRefreshFeedback();
    if isOk
        if max(abs(wrapTo360(trajangles(end,:))-actuatorFb(i).position)) < 0.2
            disp('End Point reached.')
            break;
        end 
    else
        error('SendRefreshFeedback error')
    end
    i=i+1;
end

%Communication rate is limited to 25-40Hz
for j=1:size(actuatorFb,2)
    feedbackAngles(j,:) = actuatorFb(j).position;
    feedbackVelocity(j,:) = actuatorFb(j).velocity;
end

%Save data for post processing
save('/media/goncalopereira/DATA/EXP_XAXIS_16_07_SLOW.mat');

%% Disconnect from the robot
isOk = gen3Kinova.DestroyRobotApisWrapper();
clear;
