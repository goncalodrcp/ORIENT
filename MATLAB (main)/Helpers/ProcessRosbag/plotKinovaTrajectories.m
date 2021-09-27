function plotKinovaTrajectories(gen3,sentTraj,feedbackTraj)


%jointTraj - Feedback from the actuators (ROSBAG)
%angleTraj - Trajectory at 1kHz sent to the robot

%Robot variables
numJoints = 7;
eeName = 'Gripper';

%The data structure comes from the function "compute1kHzTrajectory"
angleTraj = sentTraj.angles'*180/pi;
velTraj = sentTraj.velocity'*180/pi;
accTraj = sentTraj.acceleration'*180/pi;
timestamp = sentTraj.time;


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

%% PLOT JOINT ANGLES - PLANNING AND FEEDBACK FROM THE ACTUATORs

jointTraj = feedbackTraj.angles;
velocityTraj = feedbackTraj.velocity;
t_joints = feedbackTraj.timestamp;

 for j=1:numJoints
         if(j==1)
             jointPlot(1:numJoints) = figure;
         end
         figure(j);
         subplot(2,2,1);
         plot(t_joints-t_joints(1),jointTraj(:,j)); hold on;
         plot(timestamp,angleTraj(:,j),'-.','Linewidth',1);
         title(sprintf('Joint %d - Angle (feedback)',j))
         subplot(2,2,2);
         plot(timestamp,angleTraj(:,j));
         title(sprintf('Joint %d - Angle (Sent)',j))
         subplot(2,2,3);
         plot(t_joints-t_joints(1),velocityTraj(:,j));
         title(sprintf('Joint %d - Angular velocity (feedback)',j))
         subplot(2,2,4);
         plot(timestamp,velTraj(:,j)); 
         title(sprintf('Joint %d - Angular velocity (Sent)',j))
 end
 
 for i=1:length(jointTraj)
    FBTransform(:,:,i) = getTransform(gen3,jointTraj(i,:)*pi/180,eeName);
    eeFBPos(:,i) = tform2trvec(FBTransform(:,:,i)); %XYZ position of the end-effector
    eeFBOrientation(:,i) = tform2eul(FBTransform(:,:,i))*180/pi; %Orientation in Euler angles ZYX
    eeFBquat(:,i) = tform2quat(FBTransform(:,:,i));
 end

% Plot orientation - Euler angles
figure;
sgtitle('Orientation of the End-effector (Feedback) - Euler angles')
subplot(3,1,1);
plot(t_joints,eeFBOrientation(1,:))
title('Z-Axis');
subplot(3,1,2); 
plot(t_joints,eeFBOrientation(2,:))
title('Y-Axis');
subplot(3,1,3); 
plot(t_joints,eeFBOrientation(3,:))
title('X-Axis');

% Plot orientation - quaternion
figure;
sgtitle('Orientation of the End-effector (Feedback) - Quaternions')
subplot(4,1,1); 
plot(t_joints,eeFBquat(1,:)); %qW
title('qW');
subplot(4,1,2); 
plot(t_joints,eeFBquat(2,:)); %qX
title('qX');
subplot(4,1,3); 
plot(t_joints,eeFBquat(3,:)); %qY
title('qY');
subplot(4,1,4); 
plot(t_joints,eeFBquat(4,:)); %qZ
title('qZ');



end

