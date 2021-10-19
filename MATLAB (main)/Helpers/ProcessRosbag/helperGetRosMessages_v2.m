function [imageData,imuData,jointData] = helperGetRosMessages_v2(imageDir,imageTopic,imuTopic,jointTopic)

%Number of joints of the Kinova
numJoints = 7;

%Read messages published
%image_msgs = readMessages(image_Topic);
%image_msgs = timeseries(image_Topic);
joint_msgs = readMessages(jointTopic);
imu_msgs = readMessages(imuTopic);


numImages = imageTopic.NumMessages; %number of images
numJointMsg = jointTopic.NumMessages; %number of joint state messages
numIMUMsg = imuTopic.NumMessages; %number of imu messages

%%CAMERA
%Get all images - Some of them might be discarded later
%Create image data store object
imds = imageDatastore(imageDir);
images = imds.Files;
for i=1:numImages 
    image_msg = readMessages(imageTopic,i);
    %get timestamp
    t_images(i) = image_msg{1}.Header.Stamp.Sec +10^-9*image_msg{1}.Header.Stamp.Nsec;
end
%t_images = t_images - t_images(1);

%%KINOVA GEN3
%Store joint messages
for i=1:numJointMsg
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

%Collect image data
imageData.images = images;
imageData.timestamp = t_images;

%Collect imu data
imuData.orientation = orientationData;
imuData.gyroscope = gyroData;
imuData.accelerometer = accData;
imuData.timestamp = t_IMU;

%Collect joint data
jointData.angles = jointTraj;
jointData.velocity = velocityTraj;
jointData.timestamp = t_joints;


end

