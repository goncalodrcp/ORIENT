function [imageDataFilt,imuDataFilt,jointDataFilt] = helperPreFilterData(imageData,imuData,jointData)

numJoints = 7;

%Image data
images = imageData.images;
t_images = imageData.timestamp;

%IMU data
orientationData = imuData.orientation;
gyroData = imuData.gyroscope;
accData = imuData.accelerometer;
t_IMU = imuData.timestamp;

%Joint data
jointTraj = jointData.angles;
velocityTraj = jointData.velocity;
t_joints = jointData.timestamp;


% Use the feedback velocity from the robot to filter out samples
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
t_jointsFilt = t_joints(validSamples);
jointTrajFilt = jointTraj(validSamples,:);
velocityTrajFilt = velocityTraj(validSamples,:);
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

%Filter images and store them
filteredImages = t_images > t_jointsFilt(1) & t_images < t_jointsFilt(end);
j=1;
for i=1:length(filteredImages)
    if filteredImages(i)
        imagesFilt{j} = images{i}; %filter images
        j = j + 1;
    end
end
t_imagesFilt = t_images(filteredImages); %filter images

%Filter IMU messages
filteredIMU = t_IMU > t_jointsFilt(1) & t_IMU < t_jointsFilt(end);
orientationDataFilt = orientationData(:,filteredIMU);
gyroDataFilt = gyroData(:,filteredIMU);
accDataFilt = accData(:,filteredIMU);
t_IMUFilt = t_IMU(filteredIMU); %filter images

% Store filtered results in the structures
%Collect filtered image data
imageDataFilt.images = imagesFilt;
imageDataFilt.timestamp = t_imagesFilt;

%Collect filtered imu data
imuDataFilt.orientation = orientationDataFilt;
imuDataFilt.gyroscope = gyroDataFilt;
imuDataFilt.accelerometer = accDataFilt;
imuDataFilt.timestamp = t_IMUFilt;

%Collect filtered joint data
jointDataFilt.angles = jointTrajFilt;
jointDataFilt.velocity = velocityTrajFilt;
jointDataFilt.timestamp = t_jointsFilt;


end

