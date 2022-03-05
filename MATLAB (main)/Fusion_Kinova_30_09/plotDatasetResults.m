clear;
close all;

%% Import data for one amplitude

%note:the order of rotation is ZYX(1,2,3) or(psi,theta,phi), however, there's an offset
%rotation noticeable in the image frames and so the Y rotation is 3-theta
%and the X rotation is 2-phi
%The z axis rotation remains the same for this convention

%helper functions
addpath('D:\IST\ORIENT_repos\ORIENT\MATLAB (main)\Helpers\ProcessRosbag');
%add path to ESim test
path_to_sw = 'D:\IST\ORIENT_repos\Tests\ThesisSW\ESIM_test\Fusion_Kinova_30_09\';
addpath(genpath(path_to_sw));
dirDataset = 'D:\IST\ORIENT_repos\Tests\ThesisSW\Data collected\Experiments_14_10\Feedback';
addpath(dirDataset);

%% No filter
path = 'Results\'; 
fileName = 'res_Z_A20_v15_nofilterBAD.mat';
load([path fileName]);

tReal_NF = tReal;
% Setup variables
t_IMU_NF= data.imuData.timestamp;
%Correct time offset
tStart = t_IMU_NF(1);
t_IMU_NF = t_IMU_NF-tStart;
accData_NF = data.imuData.accelerometer;
gyroData_NF = data.imuData.gyroscope;
gyroData_NF = roty(-90)*rotx(180)*gyroData_NF;
accData_NF = roty(-90)*rotx(180)*accData_NF;
orientationData_NF = data.imuData.orientation;
orientationData_NF = orientationData_NF';
orientationIMU_euler_NF = quat2eul(orientationData_NF)*180/pi;
orientationIMU_euler_NF = roty(-90)*rotx(180)*orientationIMU_euler_NF';
%Size of the sample
Nmax_NF = NbStepsMax;
N_NF = length(tReal_NF);
%Relevant trajectories to analyse
trajSE3_NF = estTrajectories.trajL;
%Ground truth trajectory
trajGT_NF = IMU_img_struct.trajReal;
%Time instants
t_NF = IMU_img_struct.t;
%Compute errors
errorSE3_NF = computeError(trajSE3_NF,trajGT_NF,N_NF);

% Estimated orientation in euler angles (º)
trajSE3_NF.psi = trajSE3_NF.psi*180/pi;
trajSE3_NF.theta = trajSE3_NF.theta*180/pi;
trajSE3_NF.phi = trajSE3_NF.phi*180/pi;

% Estimated position (m)
estPosSE3_NF = trajSE3_NF.x;

%Grount-truth
trajGT_NF.psi = trajGT_NF.psi*180/pi;
trajGT_NF.theta = trajGT_NF.theta*180/pi;
trajGT_NF.phi = trajGT_NF.phi*180/pi;
PositionGT_NF = trajGT_NF.x;
tGT_NF = IMU_img_struct.tReal/10^9;

%% MF10

fileName = 'res_Z_A20_v15_MF10.mat';
load([path fileName]);

tReal_MF10 = tReal;
% Setup variables
t_IMU_MF10= data.imuData.timestamp;
%Correct time offset
tStart = t_IMU_MF10(1);
t_IMU_MF10 = t_IMU_MF10-tStart;
accData_MF10 = data.imuData.accelerometer;
gyroData_MF10 = data.imuData.gyroscope;
gyroData_MF10= roty(-90)*rotx(180)*gyroData_MF10;
accData_MF10 = roty(-90)*rotx(180)*accData_MF10;
orientationData_MF10 = data.imuData.orientation;
orientationData_MF10 = orientationData_MF10';
orientationIMU_euler_MF10 = quat2eul(orientationData_MF10)*180/pi;
orientationIMU_euler_MF10 = roty(-90)*rotx(180)*orientationIMU_euler_MF10';
%Size of the sample
Nmax_MF10 = NbStepsMax;
N_MF10 = length(tReal_MF10);
%Relevant trajectories to analyse
trajSE3_MF10 = estTrajectories.trajL;
%Ground truth trajectory
trajGT_MF10 = IMU_img_struct.trajReal;
%Time instants
t_MF10 = IMU_img_struct.t;
%Compute errors
errorSE3_MF10 = computeError(trajSE3_MF10,trajGT_MF10,N_MF10);

% Estimated orientation in euler angles (º)
trajSE3_MF10.psi = trajSE3_MF10.psi*180/pi;
trajSE3_MF10.theta = trajSE3_MF10.theta*180/pi;
trajSE3_MF10.phi = trajSE3_MF10.phi*180/pi;

% Estimated position (m)
estPosSE3_MF10 = trajSE3_MF10.x;

%Grount-truth
trajGT_MF10.psi = trajGT_MF10.psi*180/pi;
trajGT_MF10.theta = trajGT_MF10.theta*180/pi;
trajGT_MF10.phi = trajGT_MF10.phi*180/pi;
PositionGT_MF10 = trajGT_MF10.x;
tGT_NF = IMU_img_struct.tReal/10^9;



%% MF15

fileName = 'res_Z_A20_v15_MF15.mat';
load([path fileName]);

tReal_MF15 = tReal;
% Setup variables
t_IMU_MF15= data.imuData.timestamp;
%Correct time offset
tStart = t_IMU_MF15(1);
t_IMU_MF15 = t_IMU_MF15-tStart;
accData_MF15 = data.imuData.accelerometer;
gyroData_MF15 = data.imuData.gyroscope;
gyroData_MF15 = roty(-90)*rotx(180)*gyroData_MF15;
accData_MF15 = roty(-90)*rotx(180)*accData_MF15;
orientationData_MF15 = data.imuData.orientation;
orientationData_MF15 = orientationData_MF15';
orientationIMU_euler_MF15 = quat2eul(orientationData_MF15)*180/pi;
orientationIMU_euler_MF15 = roty(-90)*rotx(180)*orientationIMU_euler_MF15';
%Size of the sample
Nmax_MF15 = NbStepsMax;
N_MF15 = length(tReal_MF15);
%Relevant trajectories to analyse
trajSE3_MF15 = estTrajectories.trajL;
%Ground truth trajectory
trajGT_MF15 = IMU_img_struct.trajReal;
%Time instants
t_MF15 = IMU_img_struct.t;
%Compute errors
errorSE3_MF15 = computeError(trajSE3_MF15,trajGT_MF15,N_MF15);

% Estimated orientation in euler angles (º)
trajSE3_MF15.psi = trajSE3_MF15.psi*180/pi;
trajSE3_MF15.theta = trajSE3_MF15.theta*180/pi;
trajSE3_MF15.phi = trajSE3_MF15.phi*180/pi;

% Estimated position (m)
estPosSE3_MF15 = trajSE3_MF15.x;

%Grount-truth
trajGT_MF15.psi = trajGT_MF15.psi*180/pi;
trajGT_MF15.theta = trajGT_MF15.theta*180/pi;
trajGT_MF15.phi = trajGT_MF15.phi*180/pi;
PositionGT_MF10 = trajGT_NF.x;
tGT_NF = IMU_img_struct.tReal/10^9;



%% Plot IMU raw data and internal EKF estimation
plotIMUdata(t_IMU_NF,accData_NF,gyroData_NF,orientationData_NF);
plotIMUdata(t_IMU_MF10,accData_MF10,gyroData_MF10,orientationData_MF10);
plotIMUdata(t_IMU_MF15,accData_MF15,gyroData_MF15,orientationData_MF15);

figure;
sgtitle('Orientation of the IMU in Euler angles - Estimated by internal EKF')
subplot(3,1,1);
plot(t_IMU_NF,orientationIMU_euler_NF(1,:))
plot(t_IMU_MF10,orientationIMU_euler_MF10(1,:))
plot(t_IMU_MF15,orientationIMU_euler_MF15(1,:))
title('Z-Axis');
subplot(3,1,2); 
plot(t_IMU_NF,orientationIMU_euler_NF(2,:))
plot(t_IMU_MF10,orientationIMU_euler_MF10(2,:))
plot(t_IMU_MF15,orientationIMU_euler_MF15(2,:))
title('Y-Axis');
subplot(3,1,3); 
plot(t_IMU_NF,orientationIMU_euler_NF(3,:))
plot(t_IMU_MF10,orientationIMU_euler_MF10(3,:))
plot(t_IMU_MF15,orientationIMU_euler_MF15(3,:))
title('X-Axis');

%% Plot errors
%Plot orientation error(º)
figure;hold on;
plot(tReal_NF(2:N_NF-1)-tReal_NF(2),errorSE3_NF.errorR);
plot(tReal_MF10(2:N_MF10-1)-tReal_MF10(2),errorSE3_MF10.errorR);
plot(tReal_MF15(2:N_MF15-1)-tReal_MF15(2),errorSE3_MF15.errorR);
disp('RMSE(º):');
disp(sqrt(mean([errorSE3_NF.errorR errorSE3_MF10.errorR errorSE3_MF15.errorR].^2)));
disp('Max error(º):');
disp([max(errorSE3_NF.errorR) max(errorSE3_MF10.errorR) max(errorSE3_MF15.errorR)]);
legend('No filter', 'MF10','MF15')
xlabel('t (s)')
ylabel('RMSE attitdude (°)')
title('RMSE on attitude as function of time')
%print(gcf,'YAxis_A20_v18_error.png','-dpng','-r300');


%Plot error in position(m)
figure; hold on;
plot(tReal_NF(2:N_NF-1)-tReal_NF(2),errorSE3_NF.errorX);
plot(tReal_MF10(2:N_MF10-1)-tReal_MF10(2),errorSE3_MF10.errorX);
plot(tReal_MF15(2:N_MF15-1)-tReal_MF15(2),errorSE3_MF15.errorX);
disp('RMSE(m):');
disp(sqrt(mean([errorSE3_NF.errorX errorSE3_MF10.errorX errorSE3_MF15.errorX].^2)));
disp('Max error(m):');
disp([max(errorSE3_NF.errorX) max(errorSE3_MF10.errorX) max(errorSE3_MF15.errorX)]);
legend('No filter', 'MF10','MF15')
xlabel('t (s)')
ylabel('RMSE position (m)')
title('RMSE on position as function of time')

%% Compare estimated trajectories with ground truth

% Plot orientation - Euler angles
figure;
sgtitle('Orientation Euler angles')
subplot(3,1,1); 
plot(tReal_NF,trajSE3_NF.psi); hold on;
title('Z-Axis'); 
plot(tReal_MF10,trajSE3_MF10.psi);
plot(tReal_MF15,trajSE3_MF15.psi);
plot(tReal_NF,trajGT_NF.psi);
legend('No filter','MF10','MF15','Ground-truth')
xlabel('t(s)');
ylabel('Angle (º)');

subplot(3,1,2); 
plot(tReal_NF,trajSE3_NF.theta); hold on;
title('Y-Axis');
plot(tReal_MF10,trajSE3_MF10.theta);
plot(tReal_MF15,trajSE3_MF15.theta);
plot(tReal_NF,trajGT_NF.theta);
legend('No filter','MF10','MF15','Ground-truth')
xlabel('t(s)');
ylabel('Angle (º)');

subplot(3,1,3); 
plot(tReal_NF,trajSE3_NF.phi); hold on;
title('X-Axis');
plot(tReal_MF10,trajSE3_MF10.phi);
plot(tReal_MF15,trajSE3_MF15.phi);
plot(tReal_NF,trajGT_NF.phi); 
legend('No filter','MF10','MF15','Ground-truth')
xlabel('t(s)');
ylabel('Angle (º)');
%print(gcf,'YAxis_A20_v18.png','-dpng','-r300');


%% Plot zoomed in trajectories (for better visualization)

indexOfInterest_NF = (tReal_NF > 3.0) & (tReal_NF < 9.0);
indexOfInterest_MF10 = (tReal_MF10 > 3.0) & (tReal_MF10 < 9.0);
indexOfInterest_MF15 = (tReal_MF15 > 3.0) & (tReal_MF15 < 9.0);

figure;
plot(tReal_NF(indexOfInterest_NF),trajSE3_NF.theta(indexOfInterest_NF)); hold on;
title('Orientation Euler angles - Y-Axis');
plot(tReal_MF10(indexOfInterest_MF10),trajSE3_MF10.theta(indexOfInterest_MF10));
plot(tReal_MF15(indexOfInterest_MF15),trajSE3_MF15.theta(indexOfInterest_MF15));
plot(tReal_NF(indexOfInterest_NF),trajGT_NF.theta(indexOfInterest_NF));
legend('No filter','MF10','MF15','Ground-truth')
xlabel('t(s)');
ylabel('Angle (º)');
%print(gcf,'YAxis_A20_v18_zoom.png','-dpng','-r300');

