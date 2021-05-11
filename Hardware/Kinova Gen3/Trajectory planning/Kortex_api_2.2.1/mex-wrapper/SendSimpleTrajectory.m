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
%% Connection verification
% https://www.mathworks.com/help/supportpkg/robotmanipulator/ug/connect-to-gen3.html
clc
clear;
close all;

%Substitute for the real robot IP
% IP = '10.0.3.26';
% system(['ping ' IP],'-echo');



%% Create API and connect to robot

Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
gen3Kinova = kortex();
gen3Kinova.ip_address = '10.0.3.26';
gen3Kinova.user = 'admin';
gen3Kinova.password = 'admin';
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

%% Send command to robot

jointCmd = [0 15 180 230 0 55 90];

constraintType = int32(0);
speed = 0;
duration = 0;

isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
 
if isOk
    disp('Command sent to the robot. Wait for the robot to stop moving.');
else
    disp('Command error.');
end


%% Disconnect from the robot
isOk = gen3Kinova.DestroyRobotApisWrapper();
clear;