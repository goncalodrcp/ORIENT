clear;
close all;

%End effector - Base frame
posEE = [0 0 0];
orientationEE = eul2rotm([0 0 0]);
T0 = rotm2tform(orientationEE) * trvec2tform(posEE);

%IMU frame
posIMU = [0.05 0 0];
orientationIMU = [1 0 0;0 -1 0;0 0 -1];
T1 =T0*(rotm2tform(orientationIMU)*trvec2tform(posIMU)); %T1 = T'*T0

%Camera frame
posCam = [0.05 0 0];
orientationCam = eye(3);
T2 =T1*(rotm2tform(orientationCam)*trvec2tform(posCam)); %T2 = T''*T1 = T''T'T0; 

%Stack positions and orientations
pos = [posEE;tform2trvec(T1);tform2trvec(T2)];
orientation = [tform2quat(T0); ...
               tform2quat(T1); ...
               tform2quat(T2)];

% Create figure and hold it
figure
set(gcf,'Visible','on');
grid on;
plotTransforms(pos,orientation,'FrameSize',0.03);
xlim([-0.1 0.2]), ylim([-0.1 0.2]), zlim([-0.1 0.1])
hold on

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

orWaypoints = [0 pi/8 pi/4];
numWaypoints = length(orWaypoints);
% Initialize orientation of each axis
orientations = zeros(3,numWaypoints);
orientations(2,:) = orWaypoints;
 
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
waypoints = repmat(tform2trvec(T2)',1,numWaypoints); 
           
% Array of waypoint times
duration = 4; % Define total duration of the movement (s)
%timeStep = duration/(numWaypoints-1);
%waypointTimes = 0:timeStep:duration;
%waypointTimes = linspace(0,duration,numWaypoints);
waypointTimes = [0 2 4];
% Trajectory sample time
ts = 0.005; % Sampling time of the robot is 1ms
trajTimes = 0:ts:waypointTimes(end);
% Derived end effector positions in joint space trajectory
posJoint = zeros(3,numel(trajTimes));
% Derived end effector orientation in joint space 
eeEuler = zeros(numel(trajTimes),3);
% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;

%% init imu

imuFs = 100; %Hz

imu = imuSensor('accel-gyro', ...
    'ReferenceFrame', 'ENU', 'SampleRate', imuFs);

% Accelerometer
imu.Accelerometer.MeasurementRange =  19.6133;
imu.Accelerometer.Resolution = 0.0023928;
imu.Accelerometer.NoiseDensity = 0.0012356;

% Gyroscope
imu.Gyroscope.MeasurementRange = deg2rad(250);
imu.Gyroscope.Resolution = deg2rad(0.0625);
imu.Gyroscope.NoiseDensity = deg2rad(0.025);


%%  interpolation (ground truth trajectory)

interpInfo = rotationInterpolation(numWaypoints,orientations,waypointTimes,ts);

%Camera trajectory
posVector = repmat([0.1 0 0]',1,numel(trajTimes));
rotTrajectory = interpInfo.Rot';

for i=1:numel(trajTimes)
    T_IC = invSE3(rotm2tform(orientationCam)*trvec2tform(posCam));
    T_EEI = invSE3(rotm2tform(orientationIMU)*trvec2tform(posIMU));
    Tcam(:,:,i) = [quat2rotm(rotTrajectory(i,:)) posVector(:,i);0 0 0 1];
    %Tcam(:,:,i) = quat2tform(rotTrajectory(i,:))*trvec2tform(posVector(:,i)');
    T_IMU(:,:,i) = Tcam(:,:,i)*T_IC;
    T_EE(:,:,i) = Tcam(:,:,i)*T_IC*T_EEI; 
end

%End-effector trajectoy
transEE = tform2trvec(T_EE);
rotEE = tform2quat(T_EE);
%IMU trajectory
transIMU = tform2trvec(T_IMU);
rotIMU = tform2quat(T_IMU);

figure
set(gcf,'Visible','on');
grid on;
plotTransforms(tform2trvec(Tcam),tform2quat(Tcam),'FrameSize',0.03);
xlim([-0.1 0.2]), ylim([-0.1 0.2]), zlim([-0.1 0.1])
hold on
plotTransforms(transEE,rotEE,'FrameSize',0.03);
plotTransforms(transIMU,rotIMU,'FrameSize',0.03);

%Ground truth trajectory for the IMU
groundTruth = waypointTrajectory('SampleRate', imuFs, ...
    'Waypoints', transIMU, ...
    'TimeOfArrival', trajTimes, ...
    'Orientation', quaternion(rotIMU));

% Initialize the random number generator used to simulate sensor noise.
rng('default');

%Simulate IMU
for i=1:length(trajTimes)
    [truePosition(i,:), trueOrientation(i,:), ...
                trueVel(i,:), trueAcc(i,:), trueAngVel(i,:)] = groundTruth();
    [accelData(i,:), gyroData(i,:)] = imu(trueAcc(i,:), trueAngVel(i,:), ...
                trueOrientation(i,:));
    %Rotate to frame
    R = [1 0 0;0 -1 0;0 0 -1];
    accelData_rotated = R'*accelData';
    gyroData_rotated = R'*gyroData';
end
figure;
subplot(3,1,1);
plot(trajTimes,trueAcc(:,1))
hold on;
plot(trajTimes,accelData(:,1))
plot(trajTimes,accelData_rotated(1,:))
title('X-Axis');
legend('Ground-Truth','Measurement','Rotated Measurement');

subplot(3,1,2); 
plot(trajTimes,trueAcc(:,2))
hold on;
plot(trajTimes,accelData(:,2))
plot(trajTimes,accelData_rotated(2,:))
title('Y-Axis');
legend('Ground-Truth','Measurement','Rotated Measurement');

subplot(3,1,3); 
plot(trajTimes,trueAcc(:,3))
hold on;
plot(trajTimes,accelData(:,3))
plot(trajTimes,accelData_rotated(3,:))
title('Z-Axis');
legend('Ground-Truth','Measurement','Rotated Measurement');

figure;
subplot(3,1,1);
plot(trajTimes,trueAngVel(:,1))
hold on;
plot(trajTimes,gyroData(:,1))
plot(trajTimes,gyroData_rotated(1,:))
title('X-Axis');
legend('Ground-Truth','Measurement','Rotated Measurement');

subplot(3,1,2); 
plot(trajTimes,trueAngVel(:,2))
hold on;
plot(trajTimes,gyroData(:,2))
plot(trajTimes,gyroData_rotated(2,:))
title('Y-Axis');
legend('Ground-Truth','Measurement','Rotated Measurement');

subplot(3,1,3); 
plot(trajTimes,trueAngVel(:,3))
hold on;
plot(trajTimes,gyroData(:,3))
plot(trajTimes,gyroData_rotated(3,:))
title('Z-Axis');
legend('Ground-Truth','Measurement','Rotated Measurement');

%% Auxiliary functions 

function T_inv = invSE3(T)
    
  T_inv = [tform2rotm(T)' -tform2rotm(T)*tform2trvec(T)'; 0 0 0 1];

end 

