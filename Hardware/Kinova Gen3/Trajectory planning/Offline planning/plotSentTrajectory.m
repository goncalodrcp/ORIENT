function plotSentTrajectory(gen3,waypointTimes,sentTraj)

%Robot variables
numJoints = 7;
eeName = 'Gripper';

numWaypoints = length(waypointTimes);

velocityLimit = 50; %rad/s
accelerationLimit = 57.3; %rad/s^2

%The data structure comes from the function "compute1kHzTrajectory"
angleTraj = sentTraj.angles'*180/pi;
velTraj = sentTraj.velocity'*180/pi;
accTraj = sentTraj.acceleration'*180/pi;
timestamp = sentTraj.time;

%% Plot joint angles,velocities and accelerations
anglePlot = figure;
velPlot = figure;
accPlot = figure;
for idx=1:numJoints
    
    % Angle plot
    figure(anglePlot), hold on;
    plot(timestamp,angleTraj(:,idx));
    for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
    end
     title('Inverse Kinematics results - angle'); 
     xlabel('Time [s]');
     ylabel('Joint Angle [rad]');
     
     %Velocity plot
     figure(velPlot), hold on;
     plot(timestamp,velTraj(:,idx));
     yline(velocityLimit,'r','Max. Speed'); %Speed limit
     for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
     end
     title('Inverse Kinematics results - velocity'); 
     xlabel('Time [s]');
     ylabel('Angular velocity [rad/s]');
     
     % Acceleration plot
     figure(accPlot), hold on;
     plot(timestamp,accTraj(:,idx));
     yline(accelerationLimit,'r','Max. Acceleration'); %Acceleration limit
     for wIdx = 1:numWaypoints
        xline(waypointTimes(wIdx),'k--'); 
     end
     title('Inverse Kinematics results - acceleration'); 
     xlabel('Time [s]');
     ylabel('Angular acceleration [rad/s^2]');
          
end



%% Plot trajectory planning results

%Trajectory planning, expected orientation
%Sent to the kinova
for i=1:length(angleTraj)
   transform(:,:,i) = getTransform(gen3,angleTraj(i,:)*pi/180,eeName); 
   eePos(:,i) = tform2trvec(transform(:,:,i)); %XYZ position of the end-effector
   eeOrientation(:,i) = tform2eul(transform(:,:,i))*180/pi; %Orientation in Euler angles ZYX
   eeQuat(:,i) = tform2quat(transform(:,:,i));
end
% Plot orientation - euler angles
figure;
sgtitle('Orientation of the End-effector (Planning) - Euler angles')
subplot(3,1,1); hold on;
title('Z-Axis');
plot(timestamp,eeOrientation(1,:));
subplot(3,1,2); hold on;
plot(timestamp,eeOrientation(2,:));
title('Y-Axis');
subplot(3,1,3); hold on;
plot(timestamp,eeOrientation(3,:));
title('X-Axis');

% Plot orientation - quaternion
figure;
sgtitle('Orientation of the End-effector (Planning) - Quaternions')
subplot(4,1,1);
plot(timestamp,eeQuat(1,:)); %qW
title('qW');
subplot(4,1,2);
plot(timestamp,eeQuat(2,:)); %qX
title('qX');
subplot(4,1,3); 
plot(timestamp,eeQuat(3,:)); %qY
title('qY');
subplot(4,1,4); 
plot(timestamp,eeQuat(4,:)); %qZ
title('qZ');


end

