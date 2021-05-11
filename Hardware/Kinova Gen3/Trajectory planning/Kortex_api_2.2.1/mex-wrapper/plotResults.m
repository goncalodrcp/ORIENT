clear;
close all;

load('session_15_04_mov1.mat');

%Original files from MATLAB blog post
load gen3
gen3.Gravity = [0 0 -9.81];
load robotConfig2 %2nd configuration
%Load the gripper information
eeName = 'Gripper';
numJoints = 7;
numWaypoints = 2;
% Array of waypoint times
duration = 5; % Define total duration of the movement (s)
timeStep = duration/(numWaypoints-1);
waypointTimes = 0:timeStep:duration;

%% Plot generated gripper trajectory

rot = interpInfo_low.Rot;
angv = interpInfo_low.AngVel;
angacc = interpInfo_low.AngAcc;
timeInterp = ikInfo_low.time;

% Get rotational trajectory in euler angles
eulerAngles = quat2eul(rot(:,1:size(rot,2))');

% Plot trajectories generated from quaternion interpolation
plotInterpolation(timeInterp,waypointTimes,eulerAngles,angv,angacc);

%% Replay trajectory

% jointAnglesIK = ikInfo_low.jointAngles;
% jointAnglesHome = ikInfo_low.jointAngles(1,:);
% % Create figure and hold it
% figure
% set(gcf,'Visible','on');
% show(gen3, jointAnglesHome);
% xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
% hold on
%  % Loop through values at specified interval and update figure
%  count=1;
%  for i = 1:length(timeInterp)
%     poseEE = getTransform(gen3,jointAnglesIK(i,:),eeName);
%     plotTransforms(tform2trvec(poseEE),tform2quat(poseEE),'FrameSize',0.05);
%    % Display manipulator model
%    show(gen3, jointAnglesIK(i,:), 'Frames', 'off', 'PreservePlot', false);
%    title(['Trajectory at t = ' num2str(timeInterp(i))]);
%    % Update figure
%    drawnow
%    frames(count)=getframe(gcf); %store frames for a video
%    count = count + 1;
%  end

%% Plot IK

anglePlot = figure;
velPlot = figure;
accPlot = figure;

for idx=1:numJoints
    
    % Angle plot
    figure(anglePlot), hold on;
    plot(timeInterp,ikInfo_low.jointAngles(:,idx));
    for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
    end
     title('Inverse Kinematics results - angle'); 
     xlabel('Time [s]');
     ylabel('Joint Angle [rad]');
     
     %Velocity plot
     figure(velPlot), hold on;
     plot(timeInterp,ikInfo_low.jointVel(:,idx));
     for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
     end
     title('Inverse Kinematics results - velocity'); 
     xlabel('Time [s]');
     ylabel('Angular velocity [rad/s]');
     
     % Acceleration plot
     figure(accPlot), hold on;
     plot(timeInterp,ikInfo_low.jointAcc(:,idx));
     for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
     end
     title('Inverse Kinematics results - acceleration'); 
     xlabel('Time [s]');
     ylabel('Angular acceleration [rad/s^2]');
          
end

