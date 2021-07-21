%% Process Kinova results (16/07/2021)

clear;
close all

addpath('D:\IST\ORIENT_repos\Tests\KINOVA Gen3\Experiments_16_07');

load gen3
eeName = 'Gripper';
numJoints = 7;
%Load feedback from MATLAB API
load('EXP_ZAXIS_16_07_SLOW_1.mat')
%Load rosbag
bag = rosbag('EXP_ZAXIS_SLOW_16_07_1.bag');

%% READ MESSAGES FROM ROSBAG

%Select topics 
jointState_Topic = select(bag,'Topic','/my_gen3/joint_states');
image_Topic = select(bag,'Topic','/camera/image_raw');
imu_topic = select(bag,'Topic','/imu');

%Read messages published
%image_msgs = readMessages(image_Topic);
%image_msgs = timeseries(image_Topic);
joint_msgs = readMessages(jointState_Topic);
imu_msgs = readMessages(imu_topic);

numImages = image_Topic.NumMessages; %number of images
numJointMsg = jointState_Topic.NumMessages; %number of joint state messages
numIMUMsg = imu_topic.NumMessages; %number of imu messages

%%CAMERA
%Get all images - Some of them might be discarded later
for i=1:numImages 
    image_msg = readMessages(image_Topic,i);
    image = readImage(image_msg{1});
    %convert to grayscale
    image = rgb2gray(image); 
    images_{i} = im2double(image);
    %get timestamp
    t_images(i) = image_msg{1}.Header.Stamp.Sec +10^-9*image_msg{1}.Header.Stamp.Nsec;
    %figure;
    %imshow(images_{i});
end
%t_images = t_images - t_images(1);

%%KINOVA GEN3
%Store joint messages
for i=1:length(joint_msgs)
    jointTraj(i,:) = joint_msgs{i}.Position(1:numJoints); %Save 7dof joint trajectory
    velocityTraj(i,:) = joint_msgs{i}.Velocity(1:numJoints); %velocity
    t_joints(i) = joint_msgs{i}.Header.Stamp.Sec +10^-9*joint_msgs{i}.Header.Stamp.Nsec; %convert time stamp
end
%Convert joint states to degrees
jointTraj = jointTraj*180/pi;
velocityTraj = velocityTraj*180/pi;
%t_joints = t_joints - t_joints(1);

%%IMU
for i=1:numIMUMsg
    %Collect orientation (in quaternion)
    orientationData(:,i) = [imu_msgs{i}.Orientation.W; ...
                            imu_msgs{i}.Orientation.X; ...
                            imu_msgs{i}.Orientation.Y; ...
                            imu_msgs{i}.Orientation.Z];
    %Collect angular velocity (gyroscope)
    gyroData(:,i) = [imu_msgs{i}.AngularVelocity.X; ...
                     imu_msgs{i}.AngularVelocity.Y; ...
                     imu_msgs{i}.AngularVelocity.Z]; 
    %Collect acceleration (accelerometer)
    accData(:,i) = [imu_msgs{i}.LinearAcceleration.X; ...
                     imu_msgs{i}.LinearAcceleration.Y; ...
                     imu_msgs{i}.LinearAcceleration.Z]; 
    %get timestamp
    t_IMU(i) = imu_msgs{i}.Header.Stamp.Sec +10^-9*imu_msgs{i}.Header.Stamp.Nsec; %convert time stamp
end
%t_IMU = t_IMU - t_IMU(1);

%% PRE-FILTER DATA - DISCARD SAMPLES FROM WHEN THE ROBOT IS NOT MOVING, CORRECT ANGLE OFFSET, ETC

% Filter joint data to discard time instants from when the robot is not moving
 k=1;
 for i=1:size(velocityTraj,1)
     for j=1:numJoints
        if(abs(velocityTraj(i,j)) > 0.01) %threshold to discard samples (deg/s)
            validSamples(k) = i;
            k = k + 1;
            break;
        end 
     end
 end
 %Update joint state vectors
 t_joints = t_joints(validSamples);
 jointTraj = jointTraj(validSamples,:);
 velocityTraj = velocityTraj(validSamples,:);
 
 %Filter images and store them
 filteredImages = t_images > t_joints(1) & t_images < t_joints(end);
 j=1;
 for i=1:length(filteredImages)
     if filteredImages(i)
        imageData{j} = images_{i}; %filter images
        j = j + 1;
     end
 end
 t_images = t_images(filteredImages); %filter images
 
 %Correct offsets in the angles
 for i=1:size(jointTraj,2)
     for j=1:size(jointTraj,1)-1
         if(jointTraj(j+1,i)-jointTraj(j,i) > 340)
             fprintf('WARNING: Positive discontinuity at joint %d\n',i);
             jointTraj(:,i) = wrapTo360(jointTraj(:,i));
             break;
         elseif(jointTraj(j+1,i)-jointTraj(j,i) < -340)
             fprintf('WARNING: Negative discontinuity at joint %d\n',i);
             jointTraj(:,i) = wrapTo360(jointTraj(:,i));
             break;
         end
     end
 end

%Filter IMU messages
filteredIMU = t_IMU > t_joints(1) & t_IMU < t_joints(end);
orientationData = orientationData(:,filteredIMU);
gyroData = gyroData(:,filteredIMU);
accData = accData(:,filteredIMU);
t_IMU = t_IMU(filteredIMU); %filter images

%% ALIGN TIME VECTORS

% Concatenate vectors and then sort them in ascending order by timestamp
%t_new = sort([t_joints t_IMU t_images]);
% Create logic vector that maps the sampling times of each sensor
%occurrencesIMU = ismembertol(t_new,t_IMU);
%occurrencesImages = ismembertol(t_new,t_images);
%occurrencesJointAngles = ismembertol(t_new,t_joints);
% Correct offset to start at t=0
%t_new = t_new - t_new(1); %is this okay?

[flag,t_sorted,occurrences] = sortVectors(t_IMU,t_joints,t_images);
if(~flag)
   disp('Something went wrong when aligning the vectors'); 
end

 
%% PLOT JOINT ANGLES - SENT AND FEEDBACK FROM THE ACTUATORS
 
 for j=1:numJoints
         if(j==1)
             jointPlot(1:numJoints) = figure;
         end
         figure(j);
         subplot(2,2,1);
         plot(t_joints-t_joints(1),jointTraj(:,j)); 
         title(sprintf('Joint %d - Angle (feedback)',j))
         subplot(2,2,2);
         plot(timestamp,trajangles(:,j));
         title(sprintf('Joint %d - Angle (Sent)',j))
         subplot(2,2,3);
         plot(t_joints-t_joints(1),velocityTraj(:,j));
         title(sprintf('Joint %d - Angular velocity (feedback)',j))
         subplot(2,2,4);
         plot(timestamp,trajvel(:,j)); 
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

 
%% PLOT IMU DATA

% Acceleration
figure;
sgtitle('Accelerometer')
subplot(3,1,1);
plot(t_IMU,accData(1,:))
title('X-Axis');
subplot(3,1,2); 
plot(t_IMU,accData(2,:))
title('Y-Axis');
subplot(3,1,3); 
plot(t_IMU,accData(3,:))
title('Z-Axis');

% Angular Velocity
figure;
sgtitle('Gyroscope')
subplot(3,1,1);
plot(t_IMU,gyroData(1,:))
title('X-Axis');
subplot(3,1,2); 
plot(t_IMU,gyroData(2,:))
title('Y-Axis');
subplot(3,1,3); 
plot(t_IMU,gyroData(3,:))
title('Z-Axis');

%Estimated orientation in Euler angles - Embedded EKF
orientationData = orientationData';
orientationIMU_euler = quat2eul(orientationData)*180/pi;
figure;
sgtitle('Orientation of the IMU in Euler angles - Estimated by internal EKF')
subplot(3,1,1);
plot(t_IMU,orientationIMU_euler(:,1))
title('Z-Axis');
subplot(3,1,2); 
plot(t_IMU,orientationIMU_euler(:,2))
title('Y-Axis');
subplot(3,1,3); 
plot(t_IMU,orientationIMU_euler(:,3))
title('X-Axis');

%Estimated orientation in quaternions - Embedded EKF
figure;
sgtitle('Orientation of the IMU in quaternions - Estimated by internal EKF')
subplot(4,1,1); 
plot(orientationData(:,1)); %qW
title('qW');
subplot(4,1,2); 
plot(orientationData(:,2)); %qX
title('qX');
subplot(4,1,3); 
plot(orientationData(:,3)); %qY
title('qY');
subplot(4,1,4); 
plot(orientationData(:,4)); %qZ
title('qZ');
 


%% Plot trajectory planning results

%Trajectory planning, expected orientation
%Sent to the kinova
for i=1:length(trajangles)
   transform(:,:,i) = getTransform(gen3,trajangles(i,:)*pi/180,eeName); 
   eePos(:,i) = tform2trvec(transform(:,:,i)); %XYZ position of the end-effector
   eeOrientation(:,i) = tform2eul(transform(:,:,i))*180/pi; %Orientation in Euler angles ZYX
   eeQuat(:,i) = tform2quat(transform(:,:,i));
end
% Plot orientation - euler angles
figure;
sgtitle('Orientation of the End-effector (Planning) - Euler angles')
subplot(3,1,1); hold on;
title('Z-Axis');
plot(trajTimes,eeOrientation(1,:));
subplot(3,1,2); hold on;
plot(trajTimes,eeOrientation(2,:));
title('Y-Axis');
subplot(3,1,3); hold on;
plot(trajTimes,eeOrientation(3,:));
title('X-Axis');

% Plot orientation - quaternion
figure;
sgtitle('Orientation of the End-effector (Planning) - Quaternions')
subplot(4,1,1);
plot(trajTimes,eeQuat(1,:)); %qW
title('qW');
subplot(4,1,2);
plot(trajTimes,eeQuat(2,:)); %qX
title('qX');
subplot(4,1,3); 
plot(trajTimes,eeQuat(3,:)); %qY
title('qY');
subplot(4,1,4); 
plot(trajTimes,eeQuat(4,:)); %qZ
title('qZ');

%% 
 
%corr = xcorr(jointTraj(:,1),trajangles(:,1));
%[jointTraj_a, trajangles_a] = alignsignals(jointTraj(:,1),trajangles(:,1));
f_feedback = 10;
f_traj = 1000;
[P,Q] = rat(f_traj/f_feedback);
jointTraj_resample = resample(jointTraj,P,Q);
[jointTraj_a, trajangles_a] = alignsignals(jointTraj_resample(:,1),trajangles(:,1));

% Match the measurement with the ground truth trajectory sent
for i=1:size(trajangles,1)
   for j=1:size(jointTraj,1)
       match(i,j) = norm(trajangles(i,:)-jointTraj(j,:));  
   end
end

%% Replay trajectory in kinova

% % Create figure and hold it
% figure
% set(gcf,'Visible','on');
% show(gen3, trajangles(1,:)*pi/180);
% xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
% hold on
%  % Loop through values at specified interval and update figure
%  count=1;
%  for i = 2:50:length(trajTimes)
%    plotTransforms(tform2trvec(transform(:,:,i)),tform2quat(transform(:,:,i)),'FrameSize',0.05);
%    % Display manipulator model
%    show(gen3, trajangles(i,:)*pi/180, 'Frames', 'off', 'PreservePlot', false);
%    title(['Trajectory at t = ' num2str(trajTimes(i))]);
%    % Update figure
%    drawnow
%    frames(count)=getframe(gcf); %store frames for a video
%    count = count + 1;
%  end
