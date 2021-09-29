%% Process Kinova results (24/07/2021)

clear;
close all

%addpath('D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_16_07');
%addpath('/media/goncalopereira/DATA/IST/ORIENT_repos/Tests/ThesisSW/Data collected/Experiments_24_07/Feedback');
%addpath('/media/goncalopereira/DATA/IST/ORIENT_repos/Tests/ThesisSW/Data collected/Experiments_24_07/Feedback/José');
addpath('D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_24_07\Feedback\Gonçalo');
addpath('D:\IST\ORIENT_repos\ORIENT\MATLAB (main)\Helpers\ProcessRosbag');

loadSTL;
eeName = 'Gripper';
numJoints = 7;
%Load feedback from MATLAB API
load('XAXIS_1_Feedback.mat')
%Load rosbag
bag = rosbag('24_07_XAXIS_1.bag');

%% READ MESSAGES FROM ROSBAG

%Select topics 
jointState_Topic = select(bag,'Topic','/my_gen3/joint_states');
image_Topic = select(bag,'Topic','/camera/image_raw');
imu_topic = select(bag,'Topic','/imu');

%Get ROS messages
[imageData, imuData, jointData] = helperGetRosMessages(image_Topic,imu_topic,jointState_Topic);


%% PRE-FILTER DATA - DISCARD SAMPLES FROM WHEN THE ROBOT IS NOT MOVING, CORRECT ANGLE OFFSET, ETC

%Pre filter data
[imageDataFilt,imuDataFilt,jointDataFilt] = helperPreFilterData(imageData,imuData,jointData);

%% ALIGN TIME VECTORS

jointTraj = jointDataFilt.angles;
velocityTraj = jointDataFilt.velocity;
t_joints = jointDataFilt.timestamp;

t_IMU = imuDataFilt.timestamp;

t_images = imageDataFilt.timestamp;


% Concatenate vectors and then sort them in ascending order by timestamp
t_new = sort([t_joints t_IMU t_images]);
% Create logic vector that maps the sampling times of each sensor
occurrencesIMU = ismembertol(t_new,t_IMU,eps);
occurrencesImages = ismembertol(t_new,t_images,eps);
occurrencesJointAngles = ismembertol(t_new,t_joints,eps);

t_IMU_new = t_new(occurrencesIMU);
t_joints_new = t_new(occurrencesJointAngles);
t_images_new = t_new(occurrencesImages);

% Correct offset to start at t=0
t_new = t_new - t_new(1); %is this okay?

%%

[flag,t_sorted,occurrences] = sortVectors(t_IMU,t_joints,t_images);
if(~flag)
   disp('Something went wrong when aligning the vectors'); 
end

%% SAVE DATA FOR THE FILTER

% path = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_24_07\Feedback\Gonçalo\';
% fileName = 'ZAXIS_1_filter_part1.mat';
% savefile = strcat(path,fileName);
% 
% save(savefile,'imageData','imuData','jointData','-v7.3');
% 
% fileName = 'ZAXIS_1_filter_part2.mat';
% savefile = strcat(path,fileName);
% 
% save(savefile,'gen3','imageDataFilt','imuDataFilt','jointDataFilt', ...
%      't_sorted','occurrences','trajectoryToSend','ikInfo','interpInfo','trajTimes');

%% PLOT KINOVA DATA

%sentTraj = trajectoryToSend;
%feedbackTraj = jointDataFilt;
%plotKinovaTrajectories(gen3,sentTraj,feedbackTraj);

 
%% PLOT IMU DATA

%Correct time offset
tStart = t_IMU(1);
t_IMU = t_IMU-tStart;

accData = imuDataFilt.accelerometer;
gyroData = imuDataFilt.gyroscope;
orientationData = imuDataFilt.orientation;
orientationData = orientationData';
orientationIMU_euler = quat2eul(orientationData)*180/pi;

plotIMUdata(t_IMU,accData,gyroData,orientationData);

%% Plot rotated IMU data


accRot = roty(-90)*rotx(180)*accData;
gyroRot = roty(-90)*rotx(180)*gyroData;

% Acceleration
figure;
sgtitle('Accelerometer rotated')
subplot(3,1,1);
plot(t_IMU,accRot(1,:))
title('X-Axis');
subplot(3,1,2); 
plot(t_IMU,accRot(2,:))
title('Y-Axis');
subplot(3,1,3); 
plot(t_IMU,accRot(3,:))
title('Z-Axis');

% Angular Velocity
figure;
sgtitle('Gyroscope rotated')
subplot(3,1,1);
plot(t_IMU,gyroRot(1,:))
title('X-Axis');
subplot(3,1,2); 
plot(t_IMU,gyroRot(2,:))
title('Y-Axis');
subplot(3,1,3); 
plot(t_IMU,gyroRot(3,:))
title('Z-Axis');


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

%replayTrajectoryKinova(gen3,sentTraj);

%%
% Create figure and hold it
% figure
% set(gcf,'Visible','on');
% show(gen3, angleTraj(1,:)*pi/180);
% xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
% hold on
% % Loop through values at specified interval and update figure
% %count=1;
%  for i = 2:50:length(timestamp)
%    transform(:,:,i) = getTransform(gen3,angleTraj(i,:)*pi/180,eeName); 
%    plotTransforms(tform2trvec(transform(:,:,i)),tform2quat(transform(:,:,i)),'FrameSize',0.05);
%    % Display manipulator model
%    show(gen3, angleTraj(i,:)*pi/180, 'Frames', 'off', 'PreservePlot', false);
%    title(['Trajectory at t = ' num2str(timestamp(i))]);
%    % Update figure
%    drawnow
%    %frames(count)=getframe(gcf); %store frames for a video
%    %count = count + 1;
%  end
